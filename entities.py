# START OF FILE entities.py
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional


@dataclass(frozen=True)
class Brand:
    id: str
    name: str


@dataclass(frozen=True)
class Vehicle:
    id: int
    category: str
    cost_call: float
    cost_hour: float
    cost_km: float
    capacity: int
    unloading_speed: float

    # Пример "умного" доступа (если нужно)
    @property
    def fuel_efficiency_index(self) -> float:
        # Внутренняя логика, не зависящая от решения
        return self.cost_km / self.capacity


@dataclass
class VehicleAssignment:
    """Класс для хранения РЕЗУЛЬТАТОВ решения для конкретной машины"""
    vehicle: Vehicle
    route: List[int]  # Список ID локаций
    total_time: float = 0.0  # k_it'' (время использования)
    total_dist: float = 0.0  # k_is'' (пройденный путь)
    is_active: bool = False  # w_I (используется ли вообще)

    def get_total_cost(self) -> float:
        """Легкий вывод стоимости по конкретной машине"""
        if not self.is_active:
            return 0.0
        return (self.vehicle.cost_call +
                self.vehicle.cost_hour * self.total_time +
                self.vehicle.cost_km * self.total_dist)


@dataclass
class Location:
    id: int
    name: str


@dataclass
class Store(Location):
    # m_it0 и m_it1: временное окно (в минутах от начала суток)
    time_start: int
    time_end: int
    # n_it'vb: {brand_id: {time_slot: volume}}
    demands: Dict[str, Dict[int, int]] = field(default_factory=dict)
    # n_iku: коэффициент разгрузки (куц_kn)
    unloading_coeff: float = 1.0


@dataclass
class Warehouse(Location):
    cost_per_volume: float  # w_ic
    fixed_staff_cost: float  # w_ie
    handling_speed: float  # w_iu
    # w_ib: какие бренды производит (если это завод)
    produced_brands: List[str] = field(default_factory=list)
    # Дополнительно: начальный запас по брендам на складе
    initial_stock: Dict[str, int] = field(default_factory=dict)
    is_factory: bool = False

@dataclass
class WarehouseAssignment:
    """Результаты решения для склада/РЦ"""
    warehouse: Warehouse
    is_active: bool = False  # w_I (индикатор использования)
    max_volume: int = 0  # w_iv (пиковый объем в ящиках)

    def get_warehouse_cost(self) -> float:
        """Расчет стоимости склада по формуле: (w_ic * w_iv + w_ie) * w_I"""
        if not self.is_active:
            return 0.0
        return (self.warehouse.cost_per_volume * self.max_volume +
                self.warehouse.fixed_staff_cost)


@dataclass
class TransportNetwork:
    # Матрицы m_isj и m_itj: [from_id][to_id]
    distance_matrix: List[List[float]]
    time_matrix: List[List[int]]


@dataclass
class Scenario:
    vehicles: List[Vehicle]
    stores: List[Store]
    warehouses: List[Warehouse]
    network: TransportNetwork
    brands: List[Brand]
    bread_unit_cost: float  # cost (матожидание цены)

    @property
    def all_locations(self) -> List[Location]:
        # Важно: порядок здесь должен соответствовать индексам в матрицах network
        return self.warehouses + self.stores

    @property
    def location_ids(self) -> List[int]:
        return [loc.id for loc in self.all_locations]