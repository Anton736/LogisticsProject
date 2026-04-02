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

        df_work = df_work.dropna(subset=[self.map.col_id])
        df_work = df_work[pd.to_numeric(df_work[self.map.col_id], errors='coerce').notna()]
        extractor = RateExtractor(df_ref, self.map)

        main_brand = Brand(id="B1", name="Общая продукция")

        # 1. Магазины
        stores = []
        for _, row in df_work.iterrows():
            crates_plan = NumericParser.to_int(row.get(self.map.col_demand_crates))
            stores.append(Store(
                id=NumericParser.to_int(row[self.map.col_id]),
                name=str(row[self.map.col_name]),
                time_start=TimeParser.to_minutes(row[self.map.col_window_from]),
                time_end=TimeParser.to_minutes(row[self.map.col_window_to]),
                demands={main_brand.id: {1440: crates_plan}},
            ))

        # 2. Склад
        factory = Warehouse(
            id=0, name="Главный Склад",
            cost_per_volume=extractor.get_float_value("Стоимость хранения"),
            fixed_staff_cost=extractor.get_float_value("Фикс. затраты склада"),
            handling_speed=extractor.get_float_value("Скорость погрузки") or 50.0,
            produced_brands=[main_brand.id],
            initial_stock={main_brand.id: 1_000_000},
            is_factory=True,
        )

        # 3. Транспорт
        dr = extractor.get_float_value(self.map.label_driver_rate)
        km = extractor.get_float_value(self.map.label_km_rate)
        hr = extractor.get_float_value(self.map.label_hour_rate)
        cap = NumericParser.to_int(extractor.get_float_value(self.map.col_veh_capacity), default=500)
        count = NumericParser.to_int(extractor.get_float_value(self.map.col_veh_count), default=10)

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
        import numpy as np
        dist = np.full((size, size), 10.0)
        np.fill_diagonal(dist, 0)
        time_ = np.full((size, size), 20)
        np.fill_diagonal(time_, 0)
        return TransportNetwork(
            distance_matrix=dist.tolist(),
            time_matrix=time_.tolist(),
        )
