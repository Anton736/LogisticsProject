# START OF FILE src/io/excel_loader.py

import pandas as pd
from src.core.entities import Scenario, Store, Warehouse, Vehicle, Brand, TransportNetwork
from src.io.base_loader import BaseDataLoader
from src.io.base_matrix_loader import BaseMatrixLoader
from src.io.excel_mapping import ExcelMapping
from src.io.parsers import TimeParser, NumericParser, RateExtractor


class LogisticsExcelLoader(BaseDataLoader):
    """
    Загружает сценарий логистики из Excel-файла.

    Единицы в модели:
      - demands[store]      → единицы продукции (реальный заказ магазина)
      - Vehicle.capacity    → ящики (физическая вместимость кузова)
      - units_per_crate     → коэффициент пересчёта (хранится в Scenario)

    Две переменные доставки (создаются в VarManager):
      - delivered_units[v,s,b] → реально доставленные единицы → выручка
      - delivered_crates[v,s,b] → ceil(units / units_per_crate) → вместимость
    """

    def __init__(
        self,
        file_path: str,
        mapping: ExcelMapping,
        matrix_loader: BaseMatrixLoader | None = None,
    ):
        self.file_path = file_path
        self.map = mapping
        self.matrix_loader = matrix_loader

    def load_scenario(self) -> Scenario:
        with pd.ExcelFile(self.file_path) as xls:
            df_work = pd.read_excel(xls, self.map.sheet_work)
            df_ref = pd.read_excel(xls, self.map.sheet_ref)

            df_work = df_work.dropna(subset=[self.map.col_code])
            extractor = RateExtractor(df_ref, self.map)

            def get_val_from_col(df, col_name, default=1.0):
                # Ищем точное или очень похожее название столбца (убираем лишние пробелы)
                for col in df.columns:
                    if str(col).strip().lower() == col_name.strip().lower():
                        series = df[col].dropna()
                        if not series.empty:
                            val = series.iloc[0]
                            # Очищаем от мусора: "40,00 ₽" -> "40.00"
                            if isinstance(val, str):
                                cleaned = ''.join(c for c in val if c.isdigit() or c in '.,-')
                                val = float(cleaned.replace(',', '.')) if cleaned else default
                            return float(val)
                return default



            units_in_crate = int(get_val_from_col(df_ref, self.map.col_units_in_crate, default=1.0))
            print(f"  [Справочник] Единиц в ящике: {units_in_crate}")

            main_brand = Brand(id="B1", name="Общая продукция")

            # 1. Магазины — спрос в ЕДИНИЦАХ продукции
            stores = []
            for _, row in df_work.iterrows():
                # Приоритет: col_demand_qty (единицы).
                # Запасной: ящики × units_in_crate.
                units_plan = NumericParser.to_int(row.get(self.map.col_demand_qty))
                if units_plan == 0:
                    crates_plan = NumericParser.to_int(row.get(self.map.col_demand_crates))
                    units_plan = crates_plan * units_in_crate

                raw_service_time = row.get(self.map.col_unloading_time)
                service_minutes = TimeParser.to_minutes(raw_service_time)
                if service_minutes == 0:
                    service_minutes = 3

                raw_code = str(row.get(self.map.col_code, "")).strip()
                if raw_code.endswith(".0"):
                    raw_code = raw_code[:-2]

                t_end = TimeParser.to_minutes(row[self.map.col_window_to])
                if t_end == 0:
                    t_end = 1440

                stores.append(Store(
                    id=raw_code,
                    name=str(row[self.map.col_name]),
                    time_start=TimeParser.to_minutes(row[self.map.col_window_from]),
                    time_end=t_end,
                    service_time=service_minutes,
                    demands={main_brand.id: {1440: units_plan}},  # единицы продукции
                ))

            # 2. Склад — initial_stock в единицах (большой запас)
            factory = Warehouse(
                id="0", name="Главный Склад",
                cost_per_volume=extractor.get_float_value(self.map.label_wh_storage_cost) or 0.0,
                fixed_staff_cost=extractor.get_float_value(self.map.label_wh_fixed_cost) or 0.0,
                handling_speed=50.0,
                produced_brands=[main_brand.id],
                initial_stock={main_brand.id: 1_000_000},
                is_factory=True,
            )

        # 3. Транспорт — вместимость в ЯЩИКАХ (физическая)
        dr = extractor.get_float_value(self.map.label_driver_rate)
        km = extractor.get_float_value(self.map.label_km_rate)
        hr = extractor.get_float_value(self.map.label_hour_rate)

        raw_cap_crates = extractor.get_float_value(self.map.col_veh_capacity)
        cap_crates = int(raw_cap_crates) if raw_cap_crates > 0 else 500

        raw_count = extractor.get_float_value(self.map.col_veh_count)
        count = int(raw_count) if raw_count > 0 else 9

        print(f"  [Транспорт] Ставка: {dr}₽, Км: {km}₽, Час: {hr}₽.")
        print(f"  [Вместимость] {cap_crates} ящ/машину. Машин: {count}.")

        vehicles = [
            Vehicle(id=i, category="Стандарт", cost_call=dr,
                    cost_km=km, cost_hour=hr, capacity=cap_crates, unloading_speed=1.0)
            for i in range(1, count + 1)
        ]

        # 4. Транспортная сеть
        all_locs_count = len(stores) + 1
        network = (
            self.matrix_loader.load_network()
            if self.matrix_loader is not None
            else self._generate_network_stub(all_locs_count)
        )

        # Используем нашу новую функцию чтения из столбца
        price_per_unit = get_val_from_col(df_ref, self.map.col_price_per_unit, default=1.0)
        print(f"  [Цена] {price_per_unit}₽/ед. Выручка = доставленные единицы × {price_per_unit}₽")

        return Scenario(
            vehicles=vehicles,
            stores=stores,
            warehouses=[factory],
            brands=[main_brand],
            network=network,
            bread_unit_cost=price_per_unit,
            units_per_crate=units_in_crate,
        )

    @staticmethod
    def _generate_network_stub(size: int) -> TransportNetwork:
        loc_ids = [str(i) for i in range(size)]
        distance_matrix = {i: {j: (0.0 if i == j else 10.0) for j in loc_ids} for i in loc_ids}
        time_matrix = {i: {j: (0 if i == j else 20) for j in loc_ids} for i in loc_ids}
        return TransportNetwork(distance_matrix=distance_matrix, time_matrix=time_matrix)