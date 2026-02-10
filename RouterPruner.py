# START OF FILE RouterPruner.py
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Set, FrozenSet, Optional
from entities import Scenario, Location, Store, Warehouse


# ---------------------------------------------------------------------------
# Конфигурация обрезки маршрутов Завод → РЦ
# ---------------------------------------------------------------------------

@dataclass
class DcPruningConfig:
    """
    Управляет эвристической обрезкой дуг Завод→РЦ.
    Обрезка безопасна: удаляет только заведомо неоптимальные маршруты.

    Атрибуты:
        enabled:              Главный выключатель.
        single_dominance:     Обрезать F2→W если ∃ F1: brands(F1) ⊇ brands(F2)
                              AND dist(F1, W) ≤ dist(F2, W) × (1 - threshold).
        composite_dominance:  Обрезать F2→W если ∃ пара (F1a, F1b):
                              brands(F1a)∪brands(F1b) ⊇ brands(F2)
                              AND dist(F1a, W) ≤ ... AND dist(F1b, W) ≤ ...
                              (обе фабрики ближе — консервативное условие).
        distance_threshold:   Минимальный выигрыш по расстоянию (0.25 = 25% ближе).
    """
    enabled: bool = True
    single_dominance: bool = True
    composite_dominance: bool = True
    distance_threshold: float = 0.25


# ---------------------------------------------------------------------------
# Основной класс
# ---------------------------------------------------------------------------

class RoutePruner:
    def __init__(self,
                 scenario: Scenario,
                 min_k_neighbors: int = 70,
                 max_k_neighbors: int = 150,
                 min_time_radius: int = 60,
                 dc_pruning: Optional[DcPruningConfig] = None):
        """
        Args:
            dc_pruning: None = DcPruningConfig() (enabled=True).
                        DcPruningConfig(enabled=False) — отключить обрезку.
        """
        self.scenario = scenario
        self.min_k = min_k_neighbors
        self.max_k = max_k_neighbors
        self.min_time_radius = min_time_radius
        self.dc_pruning = dc_pruning if dc_pruning is not None else DcPruningConfig()

        self._neighbor_cache: Dict[int, Set[int]] = self._build_adaptive_neighbor_cache()
        self._pruned_factory_dc_pairs: Set[Tuple[int, int]] = self._compute_pruned_factory_dc_pairs()

    # -----------------------------------------------------------------------
    # Публичный API
    # -----------------------------------------------------------------------

    def get_allowed_pairs(self) -> List[Tuple[int, int]]:
        """
        Возвращает допустимые дуги (from_id, to_id) после всех фильтров:
          1. Временная выполнимость.
          2. Обрезка доминируемых дуг Завод→РЦ.
          3. Адаптивная плотность для магазинов.
        """
        allowed = []
        for i in self.scenario.all_locations:
            for j in self.scenario.all_locations:
                if i.id == j.id:
                    continue
                if not self._is_time_feasible(i, j):
                    continue
                if (i.id, j.id) in self._pruned_factory_dc_pairs:
                    continue
                if isinstance(i, Warehouse) or isinstance(j, Warehouse):
                    allowed.append((i.id, j.id))
                    continue
                if j.id in self._neighbor_cache.get(i.id, set()):
                    allowed.append((i.id, j.id))
        return allowed

    def get_dc_pruning_report(self) -> str:
        """Читаемый отчёт о обрезанных дугах."""
        if not self._pruned_factory_dc_pairs:
            return "DC pruning: no arcs pruned."
        loc_by_id = {loc.id: loc for loc in self.scenario.all_locations}
        lines = [f"DC pruning: {len(self._pruned_factory_dc_pairs)} arc(s) removed:"]
        for f_id, dc_id in sorted(self._pruned_factory_dc_pairs):
            f_name  = loc_by_id.get(f_id,  type('', (), {'name': str(f_id)})()).name
            dc_name = loc_by_id.get(dc_id, type('', (), {'name': str(dc_id)})()).name
            lines.append(f"  {f_name} -> {dc_name}")
        return "\n".join(lines)

    # -----------------------------------------------------------------------
    # Обрезка Завод → РЦ
    # -----------------------------------------------------------------------

    def _compute_pruned_factory_dc_pairs(self) -> Set[Tuple[int, int]]:
        """Строит кэш обрезанных пар. Вызывается один раз в __init__."""
        pruned: Set[Tuple[int, int]] = set()
        if not self.dc_pruning.enabled:
            return pruned

        factories = [wh for wh in self.scenario.warehouses if wh.is_factory]
        dcs       = [wh for wh in self.scenario.warehouses if not wh.is_factory]
        if not factories or not dcs:
            return pruned

        dist = self.scenario.network.distance_matrix
        thr  = self.dc_pruning.distance_threshold

        for dc in dcs:
            for factory in factories:
                dist_f_dc = dist[factory.id][dc.id]
                dominated = False

                if self.dc_pruning.single_dominance and not dominated:
                    dominated = self._is_single_dominated(factory, dc, factories, dist_f_dc, thr)

                if self.dc_pruning.composite_dominance and not dominated:
                    dominated = self._is_composite_dominated(factory, dc, factories, dist_f_dc, thr)

                if dominated:
                    pruned.add((factory.id, dc.id))

        return pruned

    def _is_single_dominated(
        self,
        factory: Warehouse,
        dc: Warehouse,
        all_factories: List[Warehouse],
        dist_f_dc: float,
        threshold: float,
    ) -> bool:
        """
        Одиночное доминирование:
          ∃ F1 != F: brands(F1) ⊇ brands(F)  AND  dist(F1, DC) ≤ dist(F, DC)*(1-thr)
        """
        brands_f = frozenset(factory.produced_brands)
        dist = self.scenario.network.distance_matrix
        max_alt_dist = dist_f_dc * (1.0 - threshold)

        for alt in all_factories:
            if alt.id == factory.id:
                continue
            if frozenset(alt.produced_brands) >= brands_f:
                if dist[alt.id][dc.id] <= max_alt_dist:
                    return True
        return False

    def _is_composite_dominated(
        self,
        factory: Warehouse,
        dc: Warehouse,
        all_factories: List[Warehouse],
        dist_f_dc: float,
        threshold: float,
    ) -> bool:
        """
        Составное доминирование:
          ∃ (F1a, F1b): brands(F1a)∪brands(F1b) ⊇ brands(F)
          AND dist(F1a, DC) ≤ dist(F, DC)*(1-thr)
          AND dist(F1b, DC) ≤ dist(F, DC)*(1-thr)

        Условие "оба ближе" — консервативное: не делаем предположений о
        стоимости двух рейсов против одного, отсекаем только случай когда
        F заведомо дальше обоих альтернатив.
        """
        brands_f = frozenset(factory.produced_brands)
        dist = self.scenario.network.distance_matrix
        max_alt_dist = dist_f_dc * (1.0 - threshold)

        # Все альтернативы, которые ближе к DC
        closer = [
            alt for alt in all_factories
            if alt.id != factory.id and dist[alt.id][dc.id] <= max_alt_dist
        ]

        for i, f1a in enumerate(closer):
            for f1b in closer[i + 1:]:
                combined = frozenset(f1a.produced_brands) | frozenset(f1b.produced_brands)
                if combined >= brands_f:
                    return True
        return False

    # -----------------------------------------------------------------------
    # Адаптивная плотность для магазин→магазин
    # -----------------------------------------------------------------------

    def _build_adaptive_neighbor_cache(self) -> Dict[int, Set[int]]:
        cache = {}
        store_ids  = [s.id for s in self.scenario.stores]
        time_matrix = self.scenario.network.time_matrix

        for i_id in store_ids:
            candidates = sorted(
                [(time_matrix[i_id][j_id], j_id) for j_id in store_ids if j_id != i_id],
                key=lambda x: x[0]
            )
            selected = list(candidates[:self.min_k])

            if selected and selected[-1][0] < self.min_time_radius:
                for t_time, n_id in candidates[self.min_k:]:
                    if len(selected) >= self.max_k:
                        break
                    if t_time > self.min_time_radius:
                        break
                    selected.append((t_time, n_id))

            cache[i_id] = {x[1] for x in selected}

        return cache

    # -----------------------------------------------------------------------
    # Вспомогательные
    # -----------------------------------------------------------------------

    def _is_time_feasible(self, i: Location, j: Location) -> bool:
        start_i = i.time_start if isinstance(i, Store) else 0
        end_j   = j.time_end   if isinstance(j, Store) else 1440
        travel  = self.scenario.network.time_matrix[i.id][j.id]
        return start_i + travel <= end_j