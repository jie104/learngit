/*
 * lms_structs.h
 *
 *  Author: Konrad Banachowicz
 *          Mike Purvis <mpurvis@clearpathrobotics.com>
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef LMS5XX_LMS_STRUCTS_H_
#define LMS5XX_LMS_STRUCTS_H_

#include <stdint.h>

namespace lms5xx{
/*!
* @class scanCfg
* @brief Structure containing scan configuration.
*
* @author Konrad Banachowicz
*/
struct scanCfg
{
    /*!
     * @brief Scanning frequency.
     * 1/100 Hz
     */
    int scaningFrequency;

    /*!
     * @brief Scanning resolution.
     * 1/10000 degree
     */
    int angleResolution;

    /*!
     * @brief Start angle.
     * 1/10000 degree
     */
    int startAngle;

    /*!
     * @brief Stop angle.
     * 1/10000 degree
     */
    int stopAngle;
};

/*!
* @class scanDataCfg
* @brief Structure containing scan data configuration.
*
* @author Konrad Banachowicz
*/
struct scanDataCfg
{

    /*!
     * @brief Output channels.
     * Defines which output channel is activated.
     */
    int outputChannel;

    /*!
     * @brief Remission.
     * Defines whether remission values are output.
     */
    bool remission;

    /*!
     * @brief Remission resolution.
     * Defines whether the remission values are output with 8-bit or 16bit resolution.
     */
    int resolution;

    /*!
     * @brief Encoders channels.
     * Defines which output channel is activated.
     */
    int encoder;

    /*!
     * @brief Position.
     * Defines whether position values are output.
     */
    bool position;

    /*!
     * @brief Device name.
     * Determines whether the device name is to be output.
     */
    bool deviceName;

    bool timestamp;

    /*!
     * @brief Output interval.
     * Defines which scan is output.
     *
     * 01 every scan\n
     * 02 every 2nd scan\n
     * ...\n
     * 50000 every 50000th scan
     */
    int outputInterval;
};

struct ScanEchoCfg{
    int8_t echo_cfg = 0;
};

/*!
* @class outputRange
* @brief Structure containing scan output range configuration
*
* @author wpd
*/
struct scanOutputRange
{
    /*!
     * @brief Scanning resolution.
     * 1/10000 degree
     */
    int angleResolution;

    /*!
     * @brief Start angle.
     * 1/10000 degree
     */
    int startAngle;

    /*!
     * @brief Stop angle.
     * 1/10000 degree
     */
    int stopAngle;
};

/*!
* @class scanData
* @brief Structure containing single scan message.
*
* @author Konrad Banachowicz
*/
struct scanData
{

    /*!
     * @brief Number of samples in dist1.
     *
     */
    int dist_len1;

    /*!
     * @brief Radial distance for the first reflected pulse
     *
     */
    uint16_t dist1[1142];

    /*!
     * @brief Number of samples in dist2.
     *
     */
    int dist_len2;

    /*!
     * @brief Radial distance for the second reflected pulse
     *
     */
    uint16_t dist2[1142];

    /*!
     * @brief Number of samples in rssi1.
     *
     */

    /*!
     * @brief Number of samples in dist2.
     *
     */
    int dist_len3;

    /*!
     * @brief Radial distance for the second reflected pulse
     *
     */
    uint16_t dist3[1142];
    /*!
     * @brief Number of samples in dist2.
     *
     */
    int dist_len4;

    /*!
     * @brief Radial distance for the second reflected pulse
     *
     */
    uint16_t dist4[1142];
    /*!
     * @brief Number of samples in dist2.
     *
     */
    int dist_len5;

    /*!
     * @brief Radial distance for the second reflected pulse
     *
     */
    uint16_t dist5[1142];

    /*!
     * @brief Number of samples in rssi1.
     *
     */
    int rssi_len1;

    /*!
     * @brief Remission values for the first reflected pulse
     *
     */
    uint16_t rssi1[1142];

    /*!
     * @brief Number of samples in rssi2.
     *
     */
    int rssi_len2;

    /*!
     * @brief Remission values for the second reflected pulse
     *
     */
    uint16_t rssi2[1142];
    /*!
     * @brief Number of samples in rssi2.
     *
     */
    int rssi_len3;

    /*!
     * @brief Remission values for the second reflected pulse
     *
     */
    uint16_t rssi3[1142];
    /*!
     * @brief Number of samples in rssi2.
     *
     */
    int rssi_len4;

    /*!
     * @brief Remission values for the second reflected pulse
     *
     */
    uint16_t rssi4[1142];
    /*!
     * @brief Number of samples in rssi2.
     *
     */
    int rssi_len5;

    /*!
     * @brief Remission values for the second reflected pulse
     *
     */
    uint16_t rssi5[1142];
};
}
#endif  // LMS5XX_LMS_STRUCTS_H_


