/* Lark networking 
 * Copyright (C) 2018 Tomas Hlavacek (tomas.hlavacek@akaflieg.tu-darmstadt.de)
 *
 * This file is part of Lark.
 *
 * Lark is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Lark is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Lark.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef NET_H
#define NET_H


#define LARK_DEFAULT_SSID "lark-vario"
#define LARK_MAX_STA_CONN 8
#define LARK_DEFAULT_PWD "velikychobutek"
#define NMEA_PORT 4353

#define WIFI_CONNECTED_BIT BIT0




void networking_task(void *pvParameters);

#endif

