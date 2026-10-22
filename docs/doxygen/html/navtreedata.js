/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "Secure STM32–ESP32 IoT Sensor Platform", "index.html", [
    [ "System architecture", "index.html#mainpage_overview", null ],
    [ "Firmware verification and startup", "index.html#mainpage_secure_boot", null ],
    [ "Sensor acquisition", "index.html#mainpage_sensors", null ],
    [ "Task synchronization and shared resources", "index.html#mainpage_freertos", null ],
    [ "Fault recovery and watchdog supervision", "index.html#mainpage_recovery", null ],
    [ "Authenticated UART transactions", "index.html#mainpage_uart", null ],
    [ "AWS IoT Core integration", "index.html#mainpage_aws", null ],
    [ "Local HTTPS monitoring and OTA", "index.html#mainpage_https", null ],
    [ "TLS resource management", "index.html#mainpage_modes", null ],
    [ "Fault-injection validation", "index.html#mainpage_tests", null ],
    [ "System State Machines", "a00906.html", [
      [ "STM32 Secure-Boot Flow", "a00906.html#sm_secure_boot", null ],
      [ "AES-GCM UART Transaction / ACK / NACK / Retry", "a00906.html#sm_aes_gcm", null ],
      [ "Sensor Fault-Recovery States", "a00906.html#sm_sensor_recovery", null ],
      [ "Watchdog Supervisor States", "a00906.html#sm_watchdog", null ],
      [ "ESP32 AWS to HTTPS Switching", "a00906.html#sm_aws_https", null ],
      [ "OTA State Flow", "a00906.html#sm_ota", null ],
      [ "Replay / Authentication Rejection Flow", "a00906.html#sm_replay_auth", null ]
    ] ],
    [ "Code-Derived State Machines", "a00907.html", [
      [ "1. STM32 Secure-Boot Flow — Code Derived", "a00907.html#code_sm_secure_boot", null ],
      [ "2. AES-GCM UART Transaction / ACK / NACK / Retry — Code Derived", "a00907.html#code_sm_uart", null ],
      [ "3. Sensor Fault-Recovery States — Code Derived", "a00907.html#code_sm_sensor", [
        [ "Sensor recovery overview", "a00907.html#code_sm_sensor_overview", null ],
        [ "BME680 recovery — detailed", "a00907.html#code_sm_bme680", null ],
        [ "MPU9250 recovery — detailed", "a00907.html#code_sm_mpu9250", null ],
        [ "MAX31865 recovery — detailed", "a00907.html#code_sm_max31865", null ],
        [ "PMS5003 / PM2.5 recovery — detailed", "a00907.html#code_sm_pm25", null ],
        [ "DS18B20 group recovery — detailed", "a00907.html#code_sm_ds18b20", null ]
      ] ],
      [ "4. Watchdog Supervisor — Code Derived", "a00907.html#code_sm_watchdog", null ],
      [ "5. ESP32 AWS / HTTPS Switching — Code Derived", "a00907.html#code_sm_modes", null ],
      [ "6. ESP32 OTA State Flow — Code Derived", "a00907.html#code_sm_ota", null ],
      [ "7. Replay / Authentication Rejection — Code Derived", "a00907.html#code_sm_replay", null ],
      [ "8. Multiple Concurrent Fault Injection / Supervisor Coordination — Code Derived", "a00907.html#code_sm_multi_fault", null ],
      [ "9. Bidirectional STM32CryptoLib ↔ mbedTLS AES-GCM + UART Recovery — Code Derived", "a00907.html#code_sm_uart_hw_ack_nack", null ]
    ] ],
    [ "Topics", "topics.html", "topics" ],
    [ "Data Structures", "annotated.html", [
      [ "Data Structures", "annotated.html", "annotated_dup" ],
      [ "Data Structure Index", "classes.html", null ],
      [ "Data Fields", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Variables", "functions_vars.html", "functions_vars" ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "Globals", "globals.html", [
        [ "All", "globals.html", "globals_dup" ],
        [ "Functions", "globals_func.html", "globals_func" ],
        [ "Variables", "globals_vars.html", "globals_vars" ],
        [ "Typedefs", "globals_type.html", null ],
        [ "Enumerations", "globals_enum.html", null ],
        [ "Enumerator", "globals_eval.html", "globals_eval" ],
        [ "Macros", "globals_defs.html", "globals_defs" ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"a00053.html",
"a00074.html#a771759d700fec55c40c4260140ebf9eaad823d223c01a7ae7b8a0e2c11dd93529",
"a00095.html#a470bb195d35ef7a0b75164c228b4ee69",
"a00113.html#ab83d4a40d2db8fccb60e39a89f548d6ba6e34ac5910f904124493ee1e0b348b63",
"a00140.html#adc5e8ee5417bded3f561f731b1c90224ac382bee4a33795f5bc1a6682fbd179da",
"a00191.html#aac97b3b1bfdcb8d21d7573fdcfd74763",
"a00209.html#a514a5807d084eb48770e3fef77e381e9",
"a00260.html#a8b7b8c8e75ee77ab9c783bc001f0fcd7",
"a00281.html#a8d2a65dd54a9c5f33280b32f23a5c941ab3314c58826c1224f3fcb8f246a9474e",
"a00299_source.html",
"a00329.html#a999d4d83026593b222c6802c7b5fb209aa11f4437a2ebe41cfcdd422f2f09345f",
"a00347.html",
"a00393.html#ad97ebd01754136bd9af03c9e09c43005",
"a00545.html",
"a00889.html#a6a6c1de42242f741978cb03b268e30f6",
"a00933.html#a58441e97edb3f77fc87a652cd8352e1c",
"a00942.html#af4f54c0c5c219fe3daecfcb179845a60a14a72a04e54133c4f6f902eb295834f6",
"a00945.html#aff510da6066bba16ee1d22fed58a5846aec0c74139203a6659b64e080e349893c",
"a00972.html#a34018a0690e0c1d1e453b87b339cc322",
"a01014.html#a21fb3bc304a056c1d419a1ed0dc0eee6",
"a01038.html#aac0f5707bbf1aec838b7e3aa59e8dc02",
"globals_eval_r.html"
];

const SYNCONMSG = 'click to disable panel synchronization';
const SYNCOFFMSG = 'click to enable panel synchronization';
const LISTOFALLMEMBERS = 'List of all members';