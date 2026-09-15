/**
 * Lightweight localisation: English strings are the keys; `t()` returns the translation for the
 * current language (falls back to the key). The current language is stored in localStorage.
 */
export type Lang = 'en' | 'ru';

let current: Lang = 'en';
try { const saved = localStorage.getItem('vbs.lang'); if (saved === 'ru' || saved === 'en') current = saved; } catch { /* ignore */ }

const listeners = new Set<() => void>();
export function getLang(): Lang { return current; }
export function setLang(l: Lang): void {
  current = l;
  try { localStorage.setItem('vbs.lang', l); } catch { /* ignore */ }
  for (const fn of listeners) fn();
}
export function onLangChange(fn: () => void): () => void { listeners.add(fn); return () => listeners.delete(fn); }

export function t(s: string): string {
  if (current === 'en') return s;
  const d = DICT[current];
  if (d[s]) return d[s];
  // strip trailing ellipsis / shortcut decorations for lookup
  const base = s.replace(/…$/, '');
  if (d[base]) return d[base] + (s.endsWith('…') ? '…' : '');
  return s;
}

const RU: Record<string, string> = {
  // menus
  'File': 'Файл', 'Edit': 'Правка', 'Add': 'Добавить', 'Program': 'Программа', 'Robot': 'Робот', 'Mobile & Fleet': 'Мобильные и флот', 'Agriculture': 'Агро', 'Connect': 'Подключение', 'View': 'Вид', 'Help': 'Справка',
  'New station': 'Новая станция', 'Open / import': 'Открыть / импорт', 'Save station (.vbstation)': 'Сохранить станцию (.vbstation)', 'Demo stations': 'Демо-станции',
  'Export program (post processor)': 'Экспорт программы (постпроцессор)', 'Export station as RoboDK API script (.py)': 'Экспорт станции как RoboDK API скрипт (.py)', 'Export station JSON (for rdk_export.py / server)': 'Экспорт станции JSON (для rdk_export.py / сервера)', 'Export screenshot (PNG)': 'Экспорт снимка экрана (PNG)',
  'Undo': 'Отменить', 'Redo': 'Повторить', 'Delete selection': 'Удалить выделенное', 'Rename station': 'Переименовать станцию', 'Station name': 'Имя станции',
  'Robot from library': 'Робот из библиотеки', 'Robot from online library (ROS-Industrial)': 'Робот из онлайн-библиотеки (ROS-Industrial)', 'Add robot from online library': 'Добавить робота из онлайн-библиотеки', 'Download 3D meshes': 'Скачать 3D-модели', 'Reference frame': 'Система координат', 'Target (teach)': 'Цель (обучить)', 'Target (at robot TCP)': 'Цель (в TCP робота)', 'Folder': 'Папка', 'Box': 'Параллелепипед', 'Cylinder': 'Цилиндр', 'Sphere': 'Сфера',
  'Process component (VC-style)': 'Компонент процесса (стиль VC)', 'Process component': 'Компонент процесса', 'Import URDF / STL / program / .rdk': 'Импорт URDF / STL / программы / .rdk',
  'New program': 'Новая программа', 'Teach MoveJ': 'Обучить MoveJ', 'Teach MoveL': 'Обучить MoveL', 'Run': 'Запуск', 'Pause': 'Пауза', 'Stop': 'Стоп', 'Validate (compile)': 'Проверить (компиляция)', 'Export with post processor': 'Экспорт постпроцессором',
  'Home active robot': 'Активный робот в исходное', 'Add tool to active robot': 'Добавить инструмент активному роботу', 'Fruit picking program for active robot': 'Программа сбора плодов для активного робота', 'Follow curve / points of an object': 'Следовать по кривой / точкам объекта', 'Move with external axes (rail / gantry)': 'Движение с внешними осями (рельс / портал)', 'Move with external axes (rail/gantry)': 'Движение с внешними осями (рельс/портал)',
  'Check collisions now (static)': 'Проверить коллизии сейчас (статически)', 'Check collisions during program validation': 'Проверять коллизии при проверке программы',
  'Add mobile robot': 'Добавить мобильного робота', 'Create fleet': 'Создать флот', 'Occupancy map': 'Карта проходимости', 'Zone (charging / no-go)': 'Зона (зарядка / запрет)', 'Start world simulation': 'Запустить симуляцию мира', 'Pause world': 'Пауза мира', 'Reset world': 'Сброс мира',
  'Create field / orchard': 'Создать поле / сад', 'Create mission': 'Создать миссию', 'Import field from GeoJSON': 'Импорт поля из GeoJSON', 'Demo: apple orchard with harvesting fleet': 'Демо: яблоневый сад с флотом сборщиков', 'Demo: greenhouse tomato with rail robots': 'Демо: теплица с томатами и рельсовыми роботами',
  'ROS 2 via rosbridge': 'ROS 2 через rosbridge', 'Disconnect rosbridge': 'Отключить rosbridge', 'Studio server (Python RoboDK API clients)': 'Сервер студии (Python клиенты RoboDK API)',
  'Fit all': 'Показать всё', 'Top': 'Сверху', 'Front': 'Спереди', 'Side': 'Сбоку', 'Isometric': 'Изометрия', 'Show reference frames': 'Показывать системы координат', 'Show targets': 'Показывать цели', 'Gizmo: translate': 'Манипулятор: перемещение', 'Gizmo: rotate': 'Манипулятор: вращение', 'Toggle bottom panel': 'Скрыть/показать нижнюю панель', 'Language: Русский': 'Язык: English',
  'Quick start': 'Быстрый старт', 'RoboDK API compatibility notes': 'Совместимость с RoboDK API', 'About': 'О программе',
  // context menu
  'Move robot here (MoveJ)': 'Переместить робота сюда (MoveJ)', 'Move robot here (MoveL)': 'Переместить робота сюда (MoveL)', 'Teach current position': 'Записать текущее положение', 'Set as Cartesian target': 'Сделать декартовой целью', 'Set as Joint target': 'Сделать суставной целью', 'Add MoveJ to program': 'Добавить MoveJ в программу', 'Add MoveL to program': 'Добавить MoveL в программу',
  'Set as active robot': 'Сделать активным роботом', 'Home': 'Исходное', 'Generate picking program for nearby fruit': 'Сгенерировать программу сбора ближайших плодов', 'Set as active reference': 'Сделать активной СК', 'Set as active tool': 'Сделать активным инструментом', 'Validate': 'Проверить', 'Export (post processor)': 'Экспорт (постпроцессор)',
  'Regenerate rows': 'Перегенерировать ряды', 'Build navigation map': 'Построить карту навигации', 'Plan mission (create fleet tasks)': 'Спланировать миссию (создать задачи флота)', 'Set home here': 'Задать дом здесь', 'Navigate to point': 'Ехать в точку', 'Add all mobile robots to fleet': 'Добавить всех мобильных роботов во флот',
  'Focus': 'Сфокусировать', 'Hide': 'Скрыть', 'Show': 'Показать', 'Rename': 'Переименовать', 'Delete': 'Удалить', 'Name': 'Имя', 'Tool (on active robot)': 'Инструмент (на активном роботе)', 'Mobile robot': 'Мобильный робот', 'Field / orchard': 'Поле / сад', 'Fleet': 'Флот', 'Mission': 'Миссия', 'Map (occupancy grid)': 'Карта (сетка проходимости)',
  // dialogs
  'Add robot from library': 'Добавить робота из библиотеки', 'Type': 'Тип', 'Crop': 'Культура', 'Field width (m)': 'Ширина поля (м)', 'Field length (m)': 'Длина поля (м)', 'Row spacing (m)': 'Междурядье (м)', 'Plant spacing (m)': 'Шаг растений (м)', 'Row heading (deg)': 'Направление рядов (°)', 'Headland (m)': 'Поворотная полоса (м)', 'Ripe fraction': 'Доля спелых', 'Greenhouse (indoor)': 'Теплица (закрытый грунт)', 'Build navigation map + headland zones': 'Построить карту навигации и зоны разворота',
  'Fleet name': 'Имя флота', 'Number of robots': 'Число роботов', 'Robot type': 'Тип робота', 'Allocation': 'Распределение', 'Add charging zone at start': 'Добавить зону зарядки в начале', 'Cost-based auction': 'Аукцион по стоимости', 'Nearest': 'Ближайший', 'Round robin': 'По кругу',
  'Field': 'Поле', 'Rows (e.g. 1-5,8; empty = all)': 'Ряды (например 1-5,8; пусто = все)', 'Work speed (m/s)': 'Рабочая скорость (м/с)', 'Work both sides of each row': 'Обрабатывать обе стороны ряда', 'Plan now (create fleet tasks)': 'Спланировать сейчас (создать задачи)',
  'Create occupancy map': 'Создать карту проходимости', 'Width (m)': 'Ширина (м)', 'Height (m)': 'Высота (м)', 'Length (m)': 'Длина (м)', 'Resolution (m/cell)': 'Разрешение (м/ячейка)', 'Rasterise objects footprints': 'Растеризовать габариты объектов', 'Create zone': 'Создать зону', 'Kind': 'Вид',
  'Add process component': 'Добавить компонент процесса', 'Connect output to': 'Соединить выход с', 'Feeder (creates products)': 'Источник (создаёт изделия)', 'Conveyor': 'Конвейер', 'Process / machine': 'Процесс / станок', 'Buffer / pallet': 'Буфер / паллета', 'Sink (consumes)': 'Сток (потребляет)',
  'Post processor': 'Постпроцессор', 'Download': 'Скачать', 'Cancel': 'Отмена', 'OK': 'ОК', 'Yes': 'Да',
  'Max fruit': 'Макс. плодов', 'Approach distance (mm)': 'Дистанция подхода (мм)', 'Approach': 'Подход', 'Generate fruit picking program': 'Сгенерировать программу сбора плодов',
  'Parameters': 'Параметры', 'Linear speed (mm/s)': 'Линейная скорость (мм/с)', 'Joint speed (deg/s)': 'Скорость суставов (°/с)', 'Linear accel (mm/s²)': 'Линейное ускорение (мм/с²)', 'Joint accel (deg/s²)': 'Ускорение суставов (°/с²)', 'Controller IP / ROS namespace': 'IP контроллера / ROS namespace',
  'Connect to ROS 2 (rosbridge_server)': 'Подключение к ROS 2 (rosbridge_server)', 'rosbridge websocket URL': 'URL websocket rosbridge', 'Mode': 'Режим', 'Publish rate (Hz)': 'Частота публикации (Гц)', 'Studio server': 'Сервер студии',
  // panels
  'Station': 'Станция', 'Properties': 'Свойства', 'Select an item in the tree or the 3D view.': 'Выберите элемент в дереве или в 3D-виде.', 'Simulation': 'Симуляция', 'Process': 'Процесс', 'Console (RoboDK API)': 'Консоль (RoboDK API)', 'Log': 'Журнал',
  'Joints (deg / mm)': 'Суставы (° / мм)', 'Cartesian jog (tool frame)': 'Декартов джог (СК инструмента)', 'Status': 'Состояние', 'Frames': 'Системы координат', 'Active reference frame': 'Активная система координат', 'Active tool': 'Активный инструмент', 'Teach target': 'Записать цель', 'Set active': 'Сделать активным',
  'Target': 'Цель', 'Joint target (use recorded joints)': 'Суставная цель (использовать записанные суставы)', 'Recorded joints': 'Записанные суставы', 'Reachable by active robot': 'Достижима активным роботом', 'MoveJ here': 'MoveJ сюда', 'MoveL here': 'MoveL сюда', 'Teach': 'Записать',
  'Tool': 'Инструмент', 'Tool kind': 'Вид инструмента', 'Instructions': 'Инструкции', 'Cycle time': 'Время цикла', 'Distance': 'Дистанция', 'Move type': 'Тип движения', 'Enabled': 'Включено', 'Message': 'Сообщение',
  'State': 'Состояние', 'Kinematics': 'Кинематика', 'Drive': 'Привод', 'Battery & capabilities': 'Батарея и возможности', 'Capacity (Wh)': 'Ёмкость (Вт·ч)', 'Capabilities (comma separated)': 'Возможности (через запятую)', 'ROS 2 namespace': 'ROS 2 namespace',
  'Crop row': 'Ряд культуры', 'Plants': 'Растения', 'Ripe fruit': 'Спелые плоды', 'Robots per row segment': 'Роботов на сегмент ряда', 'Component': 'Компонент', 'Output to': 'Выход на', 'Object': 'Объект', 'Colour': 'Цвет', 'Zone': 'Зона', 'Map': 'Карта',
  'Tools': 'Инструменты', 'Collision map': 'Карта коллизий', 'Measure (two selected items / item to TCP)': 'Измерить (два выбранных элемента / элемент до TCP)', 'Camera parameters': 'Параметры камеры', 'Record video of the 3D view (WebM)': 'Записать видео 3D-вида (WebM)', 'Stop video recording': 'Остановить запись видео', 'Export 3D HTML (self-contained viewer)': 'Экспорт 3D HTML (автономный просмотрщик)', 'Export scene as glTF (.glb)': 'Экспорт сцены в glTF (.glb)', 'New station tab': 'Новая вкладка станции', 'Close station tab': 'Закрыть вкладку станции',
  'Camera': 'Камера', 'Camera (on selected item)': 'Камера (на выбранном элементе)', 'Render view': 'Отрисовать вид', 'Save PNG': 'Сохранить PNG', 'Show robot reach': 'Показывать зону досягаемости',
  'Plan': 'Спланировать', 'Run world': 'Запустить мир', 'Start': 'Старт', 'Reset': 'Сброс', 'Speed': 'Скорость', 'End': 'Конец',
};

const DICT: Record<Lang, Record<string, string>> = { en: {}, ru: RU };
