from dataclasses import dataclass

@dataclass(frozen=True)
class ExcelMapping:
    # Лист "рабочее окно"
    sheet_work: str = "рабочее окно"
    col_id: str = "№"
    col_code: str = "код"
    col_name: str = "название тт"
    col_address: str = "адрес тт"
    col_coords: str = "координаты+"
    col_unloading_time: str = "временной промежуток на разгрузку"
    col_window_from: str = "окно доставки с"
    col_window_to: str = "окно доставки по"
    col_demand_qty: str = "кол-во продукции план"
    col_demand_crates: str = "кол-во ящиков план"
    col_demand_money: str = "сумма отгрузки план"

    # Лист "справочники"
    sheet_ref: str = "справочники"
    col_ref_channel: str = "канал сбыта"
    # Метки для футера (ставок)
    label_driver_rate: str = "Ставка водителя"
    label_km_rate: str = "стоимость на 1 км"
    label_hour_rate: str = "стоимость на 1 час"