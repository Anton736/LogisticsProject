# START OF FILE routing_orchestrator.py
import math
from typing import Optional, Dict, List

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

from src.core.entities import Scenario, Store, Warehouse
from src.optimization.pruner import RoutePruner
from src.models.demand import DemandManager
from src.core.enums import WarehouseCostMode
from src.models.solution import Solution
from src.io.solution_presenter import SolutionPresenter
from src.optimization.time_revenue_manager import LinearTimeRevenueManager, BaseTimeRevenueManager

# Масштаб для перевода float -> int в колбэках
SCALE = 1000


class RoutingOrchestrator:
    def __init__(self,
                 scenario: Scenario,
                 pruner: RoutePruner,

                 demand_manager: DemandManager,
                 warehouse_cost_mode: WarehouseCostMode = WarehouseCostMode.PEAK_INPUT,
                 objective_scale_factor: int = 100,
                 time_limit_per_run: int = 7200,
                 time_manager: Optional[BaseTimeRevenueManager] = None):


        self.scenario = scenario
        self.pruner = pruner
        self.demand_manager = demand_manager
        self.warehouse_cost_mode = warehouse_cost_mode
        self.objective_scale_factor = objective_scale_factor
        self.solution = SolutionPresenter(scenario)
        # Индексирование локаций для RoutingIndexManager (склады первые, потом магазины)
        self.all_locs = scenario.all_locations
        self.loc_idx: Dict[str, int] = {loc.id: i for i, loc in enumerate(self.all_locs)}
        self.num_nodes = len(self.all_locs)
        self.num_vehicles = len(scenario.vehicles)
        self.time_manager = time_manager if time_manager is not None else LinearTimeRevenueManager()
        self.time_limit_per_run = time_limit_per_run
        # Депо = первый завод
        self.depot_node = next(
            i for i, loc in enumerate(self.all_locs)
            if isinstance(loc, Warehouse) and loc.is_factory
        )

        # Предрасчёт спроса (ящики) и выручки по каждой точке
        K = scenario.units_per_crate
        self.node_demand_crates: Dict[str, int] = {}
        self.node_revenue: Dict[str, float] = {}

        for loc in self.all_locs:
            if isinstance(loc, Store):
                total_units = sum(
                    next(iter(loc.demands.get(b.id, {}).values()), 0)
                    for b in scenario.brands
                )
                self.node_demand_crates[loc.id] = math.ceil(total_units / K)

                rev = 0.0
                for b in scenario.brands:
                    units = next(iter(loc.demands.get(b.id, {}).values()), 0)
                    price = (getattr(b, 'price', None) or
                             getattr(b, 'sale_price', None) or
                             scenario.bread_unit_cost)
                    rev += units * price
                self.node_revenue[loc.id] = rev
            else:
                self.node_demand_crates[loc.id] = 0
                self.node_revenue[loc.id] = 0.0

        # Множество разрешённых пар от прунера
        self._allowed = set(pruner.get_allowed_pairs())

    # ------------------------------------------------------------------
    # Публичный API
    # ------------------------------------------------------------------

    def solve(self, epsilon: float = 1e-6, max_iterations: int = 20) -> Optional[Solution]:
        best_lambda = 1.0
        best_data: Optional[Dict] = None

        print(f"Starting Routing+Dinkelbach: epsilon={epsilon}, max_iter={max_iterations}")

        for iteration in range(max_iterations):
            print(f"\n--- Iteration {iteration + 1}, lambda={best_lambda:.8f} ---")

            data = self._solve_routing(best_lambda)
            if data is None:
                print("No solution found.")
                if best_data is None:
                    return None
                break

            cost = data['total_cost']
            revenue = data['total_revenue']

            if revenue == 0:
                print("Zero revenue, stopping.")
                if best_data is None:
                    return None
                break

            new_lambda = cost / revenue
            print(f"Cost={cost:.2f}, Revenue={revenue:.2f}, new_lambda={new_lambda:.8f}")

            data['optimal_lambda'] = new_lambda
            best_data = data

            if abs(new_lambda - best_lambda) < epsilon:
                print(f"Converged! lambda={new_lambda:.8f}")
                break

            best_lambda = new_lambda

        if best_data:
            return self._build_solution(best_data)
        return None

    # ------------------------------------------------------------------
    # Одна итерация Динкельбаха через RoutingModel
    # ------------------------------------------------------------------

    def _solve_routing(self, lambda_val: float) -> Optional[Dict]:
        manager = pywrapcp.RoutingIndexManager(
            self.num_nodes,
            self.num_vehicles,
            self.depot_node
        )
        routing = pywrapcp.RoutingModel(manager)

        dist_m = self.scenario.network.distance_matrix
        time_m = self.scenario.network.time_matrix

        # ---------- Arc cost: только km-стоимость (per vehicle) ----------
        # Стоимость времени учтена через SetSpanCostCoefficientForVehicle ниже,
        # чтобы корректно считать ожидание на временных окнах.
        for v_idx, v in enumerate(self.scenario.vehicles):
            ckm = int(v.cost_km * SCALE)

            def make_dist_cb(_ckm=ckm):
                def cb(fi, ti):
                    if routing.IsEnd(ti):
                        return 0
                    fid = self.all_locs[manager.IndexToNode(fi)].id
                    tid = self.all_locs[manager.IndexToNode(ti)].id
                    return int(dist_m[fid][tid] * _ckm)
                return cb

            cb_idx = routing.RegisterTransitCallback(make_dist_cb())
            routing.SetArcCostEvaluatorOfVehicle(cb_idx, v_idx)
            # Стоимость подачи машины
            routing.SetFixedCostOfVehicle(int(v.cost_call * SCALE), v_idx)

        # ---------- Time dimension ----------
        # Транзитное время = время езды + время обслуживания в точке отправления
        def make_time_cb():
            def cb(fi, ti):
                if routing.IsEnd(ti):
                    return 0
                fn = manager.IndexToNode(fi)
                tn = manager.IndexToNode(ti)
                loc_f = self.all_locs[fn]
                loc_t = self.all_locs[tn]
                service = int(loc_f.service_time) if isinstance(loc_f, Store) else 0
                return int(time_m[loc_f.id][loc_t.id]) + service
            return cb

        time_cb_idx = routing.RegisterTransitCallback(make_time_cb())
        # slack_max=480 мин (8 часов ожидания), horizon=1440 (сутки)
        routing.AddDimension(time_cb_idx, 480, 1440, False, "Time")
        time_dim = routing.GetDimensionOrDie("Time")
        if self.time_manager:
            self.time_manager.apply_penalties(
                manager=manager,
                routing=routing,
                time_dim=time_dim,
                all_locs=self.all_locs,
                loc_idx=self.loc_idx,
                node_revenue=self.node_revenue,
                scale=SCALE
            )
        # Временные окна магазинов
        for loc in self.all_locs:
            if isinstance(loc, Store):
                idx = manager.NodeToIndex(self.loc_idx[loc.id])
                time_dim.CumulVar(idx).SetRange(loc.time_start, loc.time_end)

        # Стоимость времени смены (ожидание + езда) per vehicle
        for v_idx, v in enumerate(self.scenario.vehicles):
            time_dim.SetSpanCostCoefficientForVehicle(
                int(v.cost_hour / 60.0 * SCALE), v_idx
            )

        # ---------- Capacity dimension ----------
        def make_demand_cb():
            def cb(fi):
                return self.node_demand_crates[self.all_locs[manager.IndexToNode(fi)].id]
            return cb

        dem_cb_idx = routing.RegisterUnaryTransitCallback(make_demand_cb())
        routing.AddDimensionWithVehicleCapacity(
            dem_cb_idx, 0,
            [v.capacity for v in self.scenario.vehicles],
            True, "Capacity"
        )

        # ---------- Disjunctions: Динкельбах через штраф за пропуск ----------
        # Суть: minimize(transport) + sum(lambda*revenue[j] для НЕпосещённых j)
        #       == minimize(transport - lambda*revenue[visited]) + const
        # Штраф за пропуск магазина = lambda * revenue[j], всегда >= 0
        for loc in self.all_locs:
           if isinstance(loc, Store):
               penalty = max(10_000_000, int(lambda_val * self.node_revenue[loc.id] * SCALE))
               routing.AddDisjunction(
                   [manager.NodeToIndex(self.loc_idx[loc.id])],
                   penalty
               )

        # ---------- Запрет дуг от прунера (магазин→магазин) ----------
        for i, loc_i in enumerate(self.all_locs):
             if isinstance(loc_i, Warehouse):
                 continue
             for j, loc_j in enumerate(self.all_locs):
                 if i == j or isinstance(loc_j, Warehouse):
                     continue
                 if (loc_i.id, loc_j.id) not in self._allowed:
                     routing.NextVar(manager.NodeToIndex(i)).RemoveValue(
                         manager.NodeToIndex(j)
                     )

        # ---------- Параметры поиска ----------
        params = pywrapcp.DefaultRoutingSearchParameters()
        params.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
        )
        params.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        params.time_limit.seconds = self.time_limit_per_run
        params.log_search = True

        solution = routing.SolveWithParameters(params)
        if solution is None:
            return None

        return self._extract_solution(manager, routing, solution)

    # ------------------------------------------------------------------
    # Извлечение маршрутов и расчёт реальных затрат/выручки
    # ------------------------------------------------------------------

    def _extract_solution(self, manager, routing, solution) -> Dict:
        dist_m = self.scenario.network.distance_matrix
        time_m = self.scenario.network.time_matrix


        vehicle_routes: Dict[int, List[str]] = {}
        arrival_times: Dict[tuple, int] = {}
        deliveries: Dict[tuple, Dict[str, int]] = {}  # (v_id, loc_id) -> {brand_id: units}
        total_cost = 0.0
        total_revenue = 0.0

        for v_idx, v in enumerate(self.scenario.vehicles):
            if not routing.IsVehicleUsed(solution, v_idx):
                continue

            route: List[str] = []
            route_dist = 0.0
            t = 0
            prev_loc = None
            index = routing.Start(v_idx)

            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                loc = self.all_locs[node]
                route.append(loc.id)

                if prev_loc is not None:
                    travel = int(time_m[prev_loc.id][loc.id])
                    service = int(prev_loc.service_time) if isinstance(prev_loc, Store) else 0
                    t += travel + service
                    route_dist += dist_m[prev_loc.id][loc.id]

                arrival_times[(v.id, loc.id)] = t

                # Доставка = полный спрос (магазин посещён → везём всё)
                if isinstance(loc, Store):
                    store_delivery = {}
                    for b in self.scenario.brands:
                        units = next(iter(loc.demands.get(b.id, {}).values()), 0)
                        store_delivery[b.id] = units
                    deliveries[(v.id, loc.id)] = store_delivery
                    total_revenue += self.time_manager.compute_revenue(loc, t, self.node_revenue[loc.id])

                prev_loc = loc
                index = solution.Value(routing.NextVar(index))

            vehicle_routes[v.id] = route

            # Время смены: от старта склада до финальной точки + её обслуживание
            if prev_loc is not None:
                service_last = int(prev_loc.service_time) if isinstance(prev_loc, Store) else 0
                shift_time = (t + service_last) / 60.0
            else:
                shift_time = 0.0

            total_cost += v.cost_call + v.cost_km * route_dist + v.cost_hour * shift_time

        return {
            'vehicle_routes': vehicle_routes,
            'arrival_times': arrival_times,
            'deliveries': deliveries,
            'total_cost': total_cost,
            'total_revenue': total_revenue,
        }

    # ------------------------------------------------------------------
    # Сборка финального Solution
    # ------------------------------------------------------------------

    def _build_solution(self, data: Dict):
        """
        Возвращает словарь совместимый с SolutionPresenter.
        SolutionPresenter нужно обновить под этот формат
        (больше нет var_manager — данные передаются напрямую).
        """
        return self.solution.build_solution({
            'vehicle_routes':    data['vehicle_routes'],    # {v_id: [loc_id, ...]}
            'arrival_times':     data['arrival_times'],     # {(v_id, loc_id): minutes}
            'deliveries':        data['deliveries'],        # {(v_id, loc_id): {brand_id: units}}
            'total_cost':        data['total_cost'],
            'total_revenue':     data['total_revenue'],
            'optimal_lambda':    data.get('optimal_lambda', 0.0),
            'node_revenue':      self.node_revenue,
            'node_demand_crates': self.node_demand_crates,
            'scenario':          self.scenario,
        })

# END OF FILE routing_orchestrator.py