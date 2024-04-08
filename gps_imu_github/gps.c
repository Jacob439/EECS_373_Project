/*
 * gps.c
 *
 *  Created on: Nov 15, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2019
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |---------------------------------------------------------------------------------
 */


#include <stdio.h>
#include <string.h>
#include "gps.h"
#include <stdint.h>
#include "main.h"



uint8_t rx_data = 0;
uint8_t rx_buffer[GPSBUFSIZE];
uint8_t rx_index = 0;
GPS_t GPS;

void GPS_print2(){
	printf("long: %f\n\r", GPS.nmea_latitude);
	printf("lat: %f\n\r", GPS.nmea_longitude);
	printf("speed: %f\n\r", GPS.speed_k);
	printf("date: %f\n\r", GPS.utc_time);
}

void GPS_print(char *data){
	char buf[GPSBUFSIZE] = {0,};
	sprintf(buf, "%s\n\r", data);
	//CDC_Transmit_FS((unsigned char *) buf, (uint16_t) strlen(buf));
}


void GPS_Init()
{
	HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
}


void GPS_UART_CallBack(){
	int counter = 0;
	while(1){
		printf("hi2\n\r");
		HAL_UART_Receive(&huart1, &rx_data, 1, 1);
		if (rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
			rx_buffer[rx_index++] = rx_data;
			//return ctr;
		} else {


			//GPS_print((char*)rx_buffer);
			GPS_print2();

			if(GPS_validate((char*) rx_buffer))
				GPS_parse((char*) rx_buffer);
			rx_index = 0;
			//memset(rx_buffer, 0, sizeof(rx_buffer));
			//return ctr++;
			counter++;
		}
		if(counter == 6){
			return;
		}
		counter++;
		//HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
	}
}


int GPS_validate(char *nmeastr){
    char check[3];
    char checkcalcstr[3];
    int i;
    int calculated_check;

    i=0;
    calculated_check=0;

    // check to ensure that the string starts with a $
    if(nmeastr[i] == '$')
        i++;
    else
        return 0;

    //No NULL reached, 82 char largest possible NMEA message, no '*' reached
    while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 82)){
        calculated_check ^= nmeastr[i];// calculate the checksum
        i++;
    }

    if(i >= 82){
        return 0;// the string was too long so return an error
    }

    if (nmeastr[i] == '*'){
        check[0] = nmeastr[i+1];    //put hex chars in check string
        check[1] = nmeastr[i+2];
        check[2] = 0;
    }
    else
        return 0;// no checksum separator found there for invalid

    sprintf(checkcalcstr,"%02X",calculated_check);
    return((checkcalcstr[0] == check[0])
        && (checkcalcstr[1] == check[1])) ? 1 : 0 ;
}

void GPS_parse(char *GPSstrParse){
    if(!strncmp(GPSstrParse, "$GPGGA", 6)){
    	if (sscanf(GPSstrParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.lock, &GPS.satelites, &GPS.hdop, &GPS.msl_altitude, &GPS.msl_units) >= 1){
    		GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
    		GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
    		return;
    	}
    }
    else if (!strncmp(GPSstrParse, "$GPRMC", 6)){
    	if(sscanf(GPSstrParse, "$GPRMC,%f,%f,%c,%f,%c,%f,%f,%d", &GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.speed_k, &GPS.course_d, &GPS.date) >= 1)
    		return;

    }
    else if (!strncmp(GPSstrParse, "$GPGLL", 6)){
        if(sscanf(GPSstrParse, "$GPGLL,%f,%c,%f,%c,%f,%c", &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.utc_time, &GPS.gll_status) >= 1)
            return;
    }
    else if (!strncmp(GPSstrParse, "$GPVTG", 6)){
        if(sscanf(GPSstrParse, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c", &GPS.course_t, &GPS.course_t_unit, &GPS.course_m, &GPS.course_m_unit, &GPS.speed_k, &GPS.speed_k_unit, &GPS.speed_km, &GPS.speed_km_unit) >= 1)
            return;
    }
}

float GPS_nmea_to_dec(float deg_coord, char nsew) {
    int degree = (int)(deg_coord/100);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}
