import pandas as pd
from typing import List, Tuple
from src.core.entities import Scenario, Store, Warehouse, Vehicle, Brand, TransportNetwork
from src.io.base_loader import BaseDataLoader
from src.io.excel_mapping import ExcelMapping
from src.io.parsers import TimeParser, CoordinateParser, RateExtractor, NumericParser


class LogisticsExcelLoader(BaseDataLoader):
    def __init__(self, file_path: str, mapping: ExcelMapping):
        self.file_path = file_path
        self.map = mapping

    def load_scenario(self) -> Scenario:
        # 1. Загрузка листов
        with pd.ExcelFile(self.file_path) as xls:
            df_work = pd.read_excel(xls, self.map.sheet_work)
            df_ref = pd.read_excel(xls, self.map.sheet_ref)

        # Убираем полностью пустые строки, если они есть
        df_work = df_work.dropna(subset=[self.map.col_id])

        # 2. Извлечение общих параметров из справочника (RateExtractor)
        extractor = RateExtractor(df_ref, self.map)
        driver_rate = extractor.get_float_value(self.map.label_driver_rate)
        km_rate = extractor.get_float_value(self.map.label_km_rate)
        hour_rate = extractor.get_float_value(self.map.label_hour_rate)

        # Параметры продукции (пока для одного общего бренда)
        price_unit = extractor.get_float_value(self.map.col_price_per_unit)
        units_in_crate = extractor.get_float_value(self.map.col_units_in_crate)

        main_brand = Brand(id="B1", name="Общая продукция")

        # 3. Парсинг Магазинов (Stores)
        stores = []
        for _, row in df_work.iterrows():
            # Потребность в ящиках (Demand)
            crates_plan = NumericParser.to_int(row.get(self.map.col_demand_crates))

            store = Store(
                id=NumericParser.to_int(row[self.map.col_id]),
                name=str(row[self.map.col_name]),
                time_start=TimeParser.to_minutes(row[self.map.col_window_from]),
                time_end=TimeParser.to_minutes(row[self.map.col_window_to]),
                demands={main_brand.id: crates_plan}
            )
            # Дополнительно можем сохранить координаты в атрибут, если расширим класс Store
            # coords = CoordinateParser.parse(row[self.map.col_coords])
            stores.append(store)

        # 4. Создание Завода (Warehouse)
        # Пока завод один и его нет в таблице, создаем его вручную
        # (в будущем можно добавить поиск координат завода в Excel)
        factory = Warehouse(
            id=0,
            name="Главный Завод",
            cost_per_volume=0.0,  # Обычно на заводе хранение не считаем
            fixed_staff_cost=0.0,
            unloading_speed=10.0,  # Быстрая погрузка на заводе
            produced_brands=[main_brand.id],
            initial_stock={main_brand.id: 999999},  # Бесконечный запас
            is_factory=True
        )

        # 5. Транспорт (Vehicles)
        # Создаем парк машин (например, 10 одинаковых машин на основе ставок из Excel)
        vehicles = []
        for i in range(1, 11):
            vehicles.append(Vehicle(
                id=i,
                category="Standard",
                cost_call=driver_rate,
                cost_hour=hour_rate,
                cost_km=km_rate,
                capacity=500,  # Вместимость в ящиках (нужно добавить в Excel позже)
                unloading_speed=1.0  # 1 ящик в минуту
            ))

        # 6. Сеть (Network)
        # Пока матриц нет, создаем пустую (заглушку)
        total_locs = len(stores) + 1  # +1 для завода
        network = TransportNetwork(
            distance_matrix=[[0.0] * total_locs for _ in range(total_locs)],
            time_matrix=[[0] * total_locs for _ in range(total_locs)]
        )

        return Scenario(
            vehicles=vehicles,
            stores=stores,
            warehouses=[factory],
            brands=[main_brand],
            network=network
        )