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

    Параметры:
        file_path:      Путь к основному файлу сценария.
        mapping:        Маппинг имён листов и столбцов.
        matrix_loader:  Загрузчик матриц расстояний и времён.
                        Если None — генерируется заглушка (только для разработки).
    """

    def __init__(
        self,
        file_path: str,
        mapping: ExcelMapping,
        matrix_loader: BaseMatrixLoader | None = None,
    ):
        self.file_path = file_path
        self.map = mapping
        self.matrix_loader = matrix_loader  # внедрение зависимости (DI)

    def load_scenario(self) -> Scenario:
        with pd.ExcelFile(self.file_path) as xls:
            df_work = pd.read_excel(xls, self.map.sheet_work)
            df_ref = pd.read_excel(xls, self.map.sheet_ref)

            # Фильтруем пустые строки по столбцу "Код" (col_code)
            df_work = df_work.dropna(subset=[self.map.col_code])
            extractor = RateExtractor(df_ref, self.map)

            main_brand = Brand(id="B1", name="Общая продукция")

            # 1. Магазины
            stores = []
            for _, row in df_work.iterrows():
                crates_plan = NumericParser.to_int(row.get(self.map.col_demand_crates))

                raw_service_time = row.get(self.map.col_unloading_time)
                service_minutes = TimeParser.to_minutes(raw_service_time)

                # Если в ячейке пусто или 0, поставим какой-то разумный минимум (например, 3 мин)
                if service_minutes == 0:
                    service_minutes = 3

                raw_code = str(row.get(self.map.col_code, "")).strip()
                if raw_code.endswith(".0"):
                    raw_code = raw_code[:-2]

                t_end = TimeParser.to_minutes(row[self.map.col_window_to])
                if t_end == 0: t_end = 1440

                stores.append(Store(
                    id=raw_code,
                    name=str(row[self.map.col_name]),
                    time_start=TimeParser.to_minutes(row[self.map.col_window_from]),
                    time_end=t_end,
                    service_time=service_minutes,  # Передаем в объект
                    demands={main_brand.id: {1440: crates_plan}},
                ))

            # 2. Склад (ID теперь строка "0", как в первой строке твоей матрицы)
            factory = Warehouse(
                id="0", name="Главный Склад",
                cost_per_volume=extractor.get_float_value(self.map.label_wh_storage_cost) or 0.0,
                fixed_staff_cost=extractor.get_float_value(self.map.label_wh_fixed_cost) or 0.0,
                handling_speed=50.0,
                produced_brands=[main_brand.id],
                initial_stock={main_brand.id: 1_000_000},
                is_factory=True,
            )

        # 3. Транспорт
        dr = extractor.get_float_value(self.map.label_driver_rate)
        km = extractor.get_float_value(self.map.label_km_rate)
        hr = extractor.get_float_value(self.map.label_hour_rate)

        # Безопасное чтение вместимости и количества
        raw_cap = extractor.get_float_value(self.map.col_veh_capacity)
        cap = int(raw_cap) if raw_cap > 0 else 500

        raw_count = extractor.get_float_value(self.map.col_veh_count)
        count = int(raw_count) if raw_count > 0 else 50

        # Выведем в консоль, чтобы ты видел, что прочитал скрипт
        print(f"  [Транспорт] Ставка: {dr}₽, Км: {km}₽, Час: {hr}₽. Создаем {count} машин по {cap} ящиков.")

        vehicles = [
            Vehicle(id=i, category="Стандарт", cost_call=dr,
                    cost_km=km, cost_hour=hr, capacity=cap, unloading_speed=1.0)
            for i in range(1, count + 1)
        ]

        # 4. Транспортная сеть
        all_locs_count = len(stores) + 1  # +1 склад
        network = (
            self.matrix_loader.load_network()
            if self.matrix_loader is not None
            else self._generate_network_stub(all_locs_count)
        )

        return Scenario(
            vehicles=vehicles,
            stores=stores,
            warehouses=[factory],
            brands=[main_brand],
            network=network,
            bread_unit_cost=extractor.get_float_value(self.map.col_price_per_unit) or 1.0,
        )

    # ------------------------------------------------------------------
    # Заглушка — используется только при matrix_loader=None
    # ------------------------------------------------------------------


    @staticmethod
    def _generate_network_stub(size: int) -> TransportNetwork:
        """Временная заглушка: 10 км и 20 мин между любыми точками."""
        distance_matrix = {}
        time_matrix = {}

        # Генерируем ключи в виде строк: "0", "1", "2"...
        loc_ids = [str(i) for i in range(size)]

        for i in loc_ids:
            distance_matrix[i] = {}
            time_matrix[i] = {}
            for j in loc_ids:
                if i == j:
                    distance_matrix[i][j] = 0.0
                    time_matrix[i][j] = 0
                else:
                    distance_matrix[i][j] = 10.0
                    time_matrix[i][j] = 20

        return TransportNetwork(
            distance_matrix=distance_matrix,
            time_matrix=time_matrix,
        )
