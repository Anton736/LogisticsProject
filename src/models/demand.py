from dataclasses import dataclass
from typing import List
from ortools.sat.python import cp_model


@dataclass(frozen=True)
class DemandStep:
    time_limit: int
    multiplier_x100: int


class DemandManager:
    def __init__(self, steps: List[DemandStep]):
        self.steps = sorted(steps, key=lambda x: x.time_limit)

    def get_segment_indicators(self, model: cp_model.CpModel, arrival_time: cp_model.IntVar,
                               loc_id: str, v_id: int) -> List[cp_model.BoolVarT]:
        """
        СОЗДАЕТ СЕТКУ ВРЕМЕНИ (Вызывать 1 раз для пары Машина-Магазин).
        Определяет, в какой временной интервал попала машина.
        """
        segments = []
        for i, step in enumerate(self.steps):
            is_in_segment = model.new_bool_var(f"seg_{i}_v{v_id}_l{loc_id}")
            lower_limit = self.steps[i - 1].time_limit if i > 0 else 0

            # Связываем переменную arrival_time с конкретным сегментом
            model.add_linear_constraint(arrival_time, lower_limit, step.time_limit - 1).OnlyEnforceIf(is_in_segment)
            segments.append(is_in_segment)

        # Гарантируем, что машина всегда находится ровно в одном сегменте
        model.add(sum(segments) == 1)
        return segments

    def apply_demand_limit(self, model: cp_model.CpModel, segments: List[cp_model.BoolVarT],
                           base_demand: int, brand_id: str, store_id: str) -> cp_model.IntVar:
        """
        ПРИМЕНЯЕТ ЛИМИТ (Вызывать для каждого бренда).
        Использует готовую сетку сегментов, чтобы выставить ограничение.
        """
        # Лимит может быть от 0 до 200% от базового (с запасом)
        max_vol_at_t = model.new_int_var(0, int(base_demand * 2), f"limit_s{store_id}_b{brand_id}")

        for i, is_in_segment in enumerate(segments):
            step = self.steps[i]
            calculated_demand = (base_demand * step.multiplier_x100) // 100
            # Если машина в сегменте i, то лимит для этого бренда = calculated_demand
            model.add(max_vol_at_t == calculated_demand).OnlyEnforceIf(is_in_segment)

        return max_vol_at_t