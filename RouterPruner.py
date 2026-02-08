from typing import List, Tuple, Dict, Set
from entities import Scenario, Location, Store, Warehouse


class RoutePruner:
    def __init__(self,
                 scenario: Scenario,
                 min_k_neighbors: int = 70,  # Твои "70 ближайших"
                 max_k_neighbors: int = 150,  # Верхний предел расширения
                 min_time_radius: int = 60):  # "Минимально разумное" время (минуты)

        self.scenario = scenario
        self.min_k = min_k_neighbors
        self.max_k = max_k_neighbors
        self.min_time_radius = min_time_radius

        # Кэш соседей строим сразу при создании класса
        self._neighbor_cache = self._build_adaptive_neighbor_cache()

    def get_allowed_pairs(self) -> List[Tuple[int, int]]:
        """
        Возвращает список только тех пар (from_id, to_id),
        которые прошли оба фильтра (плотность и время).
        """
        allowed = []

        for i in self.scenario.all_locations:
            for j in self.scenario.all_locations:
                if i.id == j.id: continue

                # 1. Проверка физической возможности успеть (Твой 3-й пункт)
                # Если приехали к открытию i -> поехали в j -> и всё равно опоздали к закрытию j
                if not self._is_time_feasible(i, j):
                    continue

                # 2. Логика Хабов (Склады/Заводы)
                # Если точка связана со складом, мы её оставляем (если она прошла проверку времени выше)
                if isinstance(i, Warehouse) or isinstance(j, Warehouse):
                    allowed.append((i.id, j.id))
                    continue

                # 3. Логика Магазинов (Адаптивная плотность)
                # Проверяем, входит ли j в список "умных" соседей i
                if j.id in self._neighbor_cache.get(i.id, set()):
                    allowed.append((i.id, j.id))

        return allowed

    def _build_adaptive_neighbor_cache(self) -> Dict[int, Set[int]]:
        """
        Реализация алгоритма адаптивной плотности:
        Берем 70 -> если мало по времени -> добираем до 150 или пока не станет далеко.
        """
        cache = {}
        store_ids = [s.id for s in self.scenario.stores]
        time_matrix = self.scenario.network.time_matrix

        for i_id in store_ids:
            # 1. Собираем всех остальных и сортируем по времени пути
            candidates = []
            for j_id in store_ids:
                if i_id == j_id: continue
                travel_time = time_matrix[i_id][j_id]
                candidates.append((travel_time, j_id))

            # Сортируем: от самых быстрых к самым долгим
            candidates.sort(key=lambda x: x[0])

            # 2. Берем обязательный минимум (70 соседей)
            selected_indices = candidates[:self.min_k]

            # 3. Проверяем "минимально разумное время"
            # Если 70-й сосед находится ближе, чем 60 минут...
            if selected_indices and selected_indices[-1][0] < self.min_time_radius:
                # ...продолжаем набирать соседей из оставшегося списка
                remaining = candidates[self.min_k:]

                for t_time, n_id in remaining:
                    # Прерываем, если достигли лимита количества (150)
                    if len(selected_indices) >= self.max_k:
                        break

                    # Прерываем, если вышли за радиус времени (стало > 60 мин)
                    if t_time > self.min_time_radius:
                        break

                    selected_indices.append((t_time, n_id))

            # Сохраняем ID отобранных соседей в множество для быстрого поиска
            cache[i_id] = {x[1] for x in selected_indices}

        return cache

    def _is_time_feasible(self, i: Location, j: Location) -> bool:
        """
        Проверка: Start_i + Travel_Time > End_j
        Если это верно, то маршрут невозможен даже теоретически.
        """
        # Для складов берем 0-1440, для магазинов их реальные окна
        start_i = i.time_start if isinstance(i, Store) else 0
        end_j = j.time_end if isinstance(j, Store) else 1440

        travel_t = self.scenario.network.time_matrix[i.id][j.id]

        # Условие: Самый ранний выезд + дорога > закрытие точки назначения
        if start_i + travel_t > end_j:
            return False

        return True