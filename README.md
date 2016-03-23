# Mini-Smart-Tracker

MiniSmartTracker это APRS трэкер с функцией SmartBeaconig, является развитием проекта - http://www.kh-gps.de/qaprs.htm использующего Arduino-библиотеку QAPRS. Оригинальная схема претерпела некоторые изменения,актуальную версию можно найти в папке pic данного проекта. Схема составлена Алексеем RA4FHE aka Koshak http://infotex58.ru/forum/index.php?topic=903.msg7553#msg7553 . Функции работы с датчиком температуры и измерение напряжения так же написаны RA4FHE. В данном проекте для работы с GPS данными использована библиотека  TinyGPS++ - http://arduiniana.org/2013/09/tinygps-a-new-view-of-global-positioning/ . Алгоритм работы SmartBeaconing позаимствован из проекта - https://github.com/stanleyseow/ArduinoTracker-MicroAPRS .

Для настройки нужно отредактировать файл config.h . В части обозначенной как //User defined part

MiniSmartTracker is APRS tracker with SmartBeaconig function is the development of the project - http://www.kh-gps.de/qaprs.htm using Arduino-library QAPRS. The original circuit has undergone some changes, the current version can be found in the folder pic of the project. The circuit is composed by Alexei RA4FHE aka Koshak http://infotex58.ru/forum/index.php?topic=903.msg7553#msg7553. Functions with a temperature sensor and voltage measurement was written by RA4FHE. In this project, to work with GPS data used library TinyGPS ++ - http://arduiniana.org/2013/09/tinygps-a-new-view-of-global-positioning/. The algorithm of SmartBeaconing borrowed from the project - https://github.com/stanleyseow/ArduinoTracker-MicroAPRS.

To set up you need to edit the file config.h. The part marked // User defined part
