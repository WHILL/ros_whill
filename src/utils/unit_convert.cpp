/*
MIT License

Copyright (c) 2019 WHILL inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/* Author: Hikaru Sugiura */

#include <stdint.h>

uint8_t convert_mps_to_whill_speed(float mps)
{ // m/s to whill speed
    float km_p_h = mps * 3.6; // m/s to km/h
    return km_p_h * 10; // WHILL 60 means 6.0 km/h 
}

uint8_t convert_mpss_to_whill_acc(float mpss)
{ // m/ss to whill speed
    double kmh_p_s = mpss * 3.6; // m/ss to km/h/s
    return kmh_p_s  /(0.4 * (32.0f / 256.0f));
}

uint8_t convert_radps_to_whill_speed(float tread, float radps){
    double mps = radps * tread / 2 * 2.0f;  // Half Tread * Turning Fix, Wheel Speed will be Max/2 in spin turning.
    return convert_mps_to_whill_speed(mps);
}
uint8_t convert_radpss_to_whill_acc(float tread, float radpss){
    double mpss = radpss * tread / 2 * 2.0f; // Half Tread * Turning Fix
    return convert_mpss_to_whill_acc(mpss);
}