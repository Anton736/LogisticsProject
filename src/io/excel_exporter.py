import pandas as pd
from src.models.solution import Solution
from src.core.entities import Store


class LogisticsExcelExporter:
    @staticmethod
    def _format_time(minutes: int) -> str:
        """Переводит минуты в формат ЧЧ:ММ"""
        if minutes == 0:
            return "00:00"
        elif minutes == 1440:
            return "24:00"
        h = minutes // 60
        m = minutes % 60
        return f"{h:02d}:{m:02d}"

    def export(self, solution: Solution, output_path: str = "data/маршруты_результат.xlsx"):
        rows = []

        for assignment in solution.vehicle_assignments:
            if not assignment.is_active:
                continue

            vehicle_id = assignment.vehicle.id
            current_time_minutes = 0
            for step in assignment.route:
                loc = step.location
                current_time_minutes += step.time_from_prev
                if isinstance(loc, Store) and current_time_minutes < loc.time_start:
                    current_time_minutes = max(loc.time_start, current_time_minutes)
                # Фиксируем время прибытия
                arrival_time_str = self._format_time(current_time_minutes)

                # 2. Затем прибавляем время на обслуживание (разгрузку), чтобы след. точка посчиталась правильно
                current_time_minutes += step.service_time

                # Если это склад, пропускаем или пишем нули. Нам интересны магазины.
                # Но чтобы было видно откуда выехала машина, оставим в таблице.
                plan_qty = 0
                time_from = "-"
                time_to = "-"

                if isinstance(loc, Store):
                    # Суммируем плановый спрос
                    for brand_demands in loc.demands.values():
                        plan_qty += sum(brand_demands.values())

                    time_from = self._format_time(loc.time_start)
                    time_to = self._format_time(loc.time_end)

                rows.append({
                    "Код": loc.id,
                    "текущий_маршрут": vehicle_id,
                    "расстояние": round(step.distance_from_prev, 2),
                    "время": step.time_from_prev,
                    "кол-во ед. продукции": step.delivered_volume,
                    "временной промежуток на разгрузку": step.service_time,
                     "кол-во ящиков текущее": step.delivered_crates,
                    "кол-во продукции план": plan_qty,
                    "окно доставки с": time_from,
                    "окно доставки по": time_to,
                    "время прибытия": arrival_time_str,  # [ИЗМЕНЕНИЕ 2] 11-я колонка
                    "адрес": getattr(loc, 'address', loc.name)
                })

        df = pd.DataFrame(rows)
        df.to_excel(output_path, index=False)
        print(f"\n[УСПЕХ] Детальные маршруты сохранены в: {output_path}")