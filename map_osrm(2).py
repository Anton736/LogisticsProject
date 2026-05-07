import pandas as pd
import folium
from folium.plugins import BeautifyIcon
import requests
import webbrowser
import os
import re

# ==========================================
# 1. НАСТРОЙКИ СЕРВЕРА
# ==========================================
OSRM_SERVER = "http://localhost:5000"

print("Загрузка данных...")
df_routes = pd.read_excel('C:/Users/user/Downloads/для логистики (3).xlsm')
df_dict = pd.read_excel('C:/Users/user/Downloads/для логистики (3).xlsm', sheet_name='рабочее окно')

# ==========================================
# 2. БРОНЕБОЙНОЕ НАЗНАЧЕНИЕ КОЛОНОК
# ==========================================
COL_CODE_ROUTES = df_routes.columns[1]      # исходный код маршрута (строка с "06 маршрут...")
COL_ROUTE_NUM = df_routes.columns[5]        # номер маршрута (тоже может быть строкой)
COL_TIME = df_routes.columns[12]            # время

COL_CODE_DICT = df_dict.columns[1]          # код в словаре
COL_NAME = df_dict.columns[2]               # название точки
COL_COORDS = df_dict.columns[4]             # координаты

# ==========================================
# 3. ФУНКЦИЯ ОЧИСТКИ: ИЗВЛЕКАЕМ ЧИСЛО ИЗ ЛЮБОЙ СТРОКИ
# ==========================================
def extract_number(s):
    """Из строки типа '06 маршрут Армавир' возвращает '6' (убирает ведущий ноль)."""
    s = str(s).strip()
    if not s or s == 'nan':
        return ''
    match = re.search(r'\d+', s)
    if match:
        return str(int(match.group()))   # '06' -> '6', '6' -> '6'
    return ''

# Применяем к колонкам маршрутов
df_routes[COL_CODE_ROUTES] = df_routes[COL_CODE_ROUTES].apply(extract_number)
df_routes[COL_ROUTE_NUM] = df_routes[COL_ROUTE_NUM].apply(extract_number)

# Удаляем строки, где не удалось извлечь номер (пустые или мусор)
df_routes = df_routes[df_routes[COL_CODE_ROUTES] != '']
df_routes = df_routes[df_routes[COL_ROUTE_NUM] != '']

# Применяем к словарю
df_dict[COL_CODE_DICT] = df_dict[COL_CODE_DICT].apply(extract_number)
df_dict = df_dict[df_dict[COL_CODE_DICT] != '']
df_dict = df_dict.drop_duplicates(subset=[COL_CODE_DICT])

# ==========================================
# 4. ОБЪЕДИНЕНИЕ (с суффиксами, чтобы избежать конфликта имён)
# ==========================================
df = pd.merge(df_routes, df_dict,
              left_on=COL_CODE_ROUTES, right_on=COL_CODE_DICT,
              how='left', suffixes=('', '_dict'))

# Удаляем строки без координат
df = df.dropna(subset=[COL_COORDS])

# Разбираем координаты
df[['Lat', 'Lon']] = df[COL_COORDS].str.split(',', expand=True).astype(float)

# ==========================================
# 5. ОБРАБОТКА И СОРТИРОВКА ПО ВРЕМЕНИ
# ==========================================
df['priority'] = df[COL_CODE_ROUTES].apply(lambda x: 0 if x == '0' else 1)

# Заменяем "0" на "00:00"
df[COL_TIME] = df[COL_TIME].astype(str).replace(['0', '0.0', 'nan', '0:00:00'], '00:00')
# Удаляем строки с пустым временем
df = df[df[COL_TIME] != 'nan']
df = df[df[COL_TIME] != '']

df['temp_sort_time'] = pd.to_datetime(df[COL_TIME], format='mixed', errors='coerce')
df = df.sort_values(by=[COL_ROUTE_NUM, 'temp_sort_time'])
df = df.drop(columns=['temp_sort_time'])

# ==========================================
# 6. ПОДГОТОВКА КАРТЫ
# ==========================================
center_lat, center_lon = df['Lat'].mean(), df['Lon'].mean()
m = folium.Map(location=[center_lat, center_lon], zoom_start=13, attribution_control=False)

colors = ['blue', 'green', 'red', 'purple', 'orange', 'darkred', 'cadetblue', 'black']
legend_items = []

# ==========================================
# 7. ПОСТРОЕНИЕ МАРШРУТОВ (OSRM) – без изменений
# ==========================================
print("Построение маршрутов через OSRM...")
for index, (route_num, group) in enumerate(df.groupby(COL_ROUTE_NUM)):
    route_color = colors[index % len(colors)]
    fg = folium.FeatureGroup(name=f"Маршрут {route_num}")
    legend_items.append(
        f'<div style="margin-bottom: 5px;">'
        f'<span style="background-color: {route_color}; width: 15px; height: 15px; '
        f'display: inline-block; vertical-align: middle; margin-right: 5px;"></span>'
        f'Маршрут {route_num}'
        f'</div>'
    )
    coords_list = [f"{row['Lon']},{row['Lat']}" for _, row in group.iterrows()]
    coords_string = ";".join(coords_list)
    url = f"{OSRM_SERVER}/route/v1/driving/{coords_string}?overview=full&geometries=geojson"
    try:
        response = requests.get(url)
        osrm_data = response.json()
        if osrm_data.get('code') != 'Ok':
            print(f"OSRM Ошибка для маршрута {route_num}: {osrm_data.get('message')}")
            continue
        route_info = osrm_data['routes'][0]
        geometry = route_info['geometry']['coordinates']
        shift_step = 0.00004
        multiplier = (index + 1) // 2 * (1 if index % 2 == 0 else -1)
        if index == 0: multiplier = 0
        lat_shift = shift_step * multiplier
        lon_shift = shift_step * multiplier
        folium_coords = [[lat + lat_shift, lon + lon_shift] for lon, lat in geometry]
        folium.PolyLine(
            locations=folium_coords,
            color=route_color,
            weight=4,
            opacity=0.8,
            tooltip=f'Маршрут: {route_num}'
        ).add_to(fg)
        legs = route_info['legs']
        total_distance_km = 0.0
        for point_idx, (row_idx, row) in enumerate(group.iterrows()):
            lat, lon = row['Lat'], row['Lon']
            arrival_time = row[COL_TIME]
            point_number = point_idx + 1
            point_name = "Стартовая" if point_idx == 0 else row[COL_NAME]
            if point_idx == 0:
                dist_from_prev_km = 0.0
            else:
                dist_from_prev_km = legs[point_idx - 1]['distance'] / 1000.0
                total_distance_km += dist_from_prev_km
            popup_html = f"""
            <div style="width: 250px; font-family: Arial;">
                <h4 style="margin-bottom: 5px; color: {route_color};">Маршрут {route_num}</h4>
                <b>Номер точки:</b> {point_number}<br>
                <b>ТТ:</b> {point_name}<br>
                <b>Время приезда:</b> {arrival_time}<br>
                <hr style="margin: 5px 0;">
                <b>От прошлой точки:</b> {dist_from_prev_km:.2f} км<br>
                <b>Пройдено от старта:</b> {total_distance_km:.2f} км
            </div>
            """
            if point_idx == 0:
                bg_color = '#28a745'
            elif point_idx == len(group) - 1:
                bg_color = '#dc3545'
            else:
                bg_color = '#007bff'
            num_icon = BeautifyIcon(
                icon_shape='marker',
                number=point_number,
                border_color='white',
                border_width=2,
                text_color='white',
                background_color=bg_color,
                inner_icon_style='font-size:12px; margin-top:0px;'
            )
            folium.Marker(
                location=[lat, lon],
                popup=folium.Popup(popup_html, max_width=300),
                tooltip=f"Точка {point_number}: {point_name}",
                icon=num_icon
            ).add_to(fg)
        fg.add_to(m)
    except Exception as e:
        print(f"Ошибка при обработке маршрута {route_num}: {e}")

# ==========================================
# 8. ЛЕГЕНДА, МЕНЮ И СОХРАНЕНИЕ
# ==========================================
folium.LayerControl(collapsed=False).add_to(m)

legend_html = f'''
<div style="
    position: fixed; 
    bottom: 20px; right: 20px; width: 180px; height: auto; 
    background-color: rgba(255, 255, 255, 0.9); 
    z-index: 9999; font-size: 14px; font-family: Arial;
    border: 2px solid grey; border-radius: 8px; padding: 10px;
    box-shadow: 2px 2px 5px rgba(0,0,0,0.3);
    ">
    <b style="display:block; margin-bottom:10px;">Легенда:</b>
    {''.join(legend_items)}
</div>
'''
m.get_root().html.add_child(folium.Element(legend_html))

map_filename = 'map_with_popups.html'
m.save(map_filename)
webbrowser.open('file://' + os.path.realpath(map_filename))
print("Готово! Карта открыта.")