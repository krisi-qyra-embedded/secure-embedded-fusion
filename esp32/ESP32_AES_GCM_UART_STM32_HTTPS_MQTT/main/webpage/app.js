/**
 * Secure Fusion dashboard client.
 * Dependency-free: no jQuery required.
 */

var seconds = null;
var otaTimerVar = null;
var wifiConnectInterval = null;
var sensorInterval = null;
var sensorPollBusy = false;

var API = {
    otaUpdate: "/OTAupdate",
    otaStatus: "/OTAstatus",
    wifiConnect: "/wifiConnect.json",
    wifiConnectStatus: "/wifiConnectStatus",
    wifiConnectInfo: "/wifiConnectInfo.json",
    wifiDisconnect: "/wifiDisconnect.json",
    localTime: "/localTime.json",
    apSSID: "/apSSID.json",
    ccs811Sensor: "/ccs811Sensor.json",
    bme280Sensor: "/bme280Sensor.json",
    stm32Telemetry: "/stm32Telemetry.json"
};

function byId(id)
{
    return document.getElementById(id);
}

function setText(id, value)
{
    var element = byId(id);

    if (element)
    {
        element.textContent = value;
    }
}

function setHTML(id, value)
{
    var element = byId(id);

    if (element)
    {
        element.innerHTML = value;
    }
}

function setPill(id, state, text)
{
    var element = byId(id);

    if (!element)
    {
        return;
    }

    element.classList.remove(
        "waiting",
        "online",
        "offline"
    );

    element.classList.add(state);
    element.textContent = text;
}

function setSecureLinkState(state, text)
{
    setText(
        "stm32_secure_status",
        text
    );

    var badge =
        byId("stm32_security_badge");

    if (!badge)
    {
        return;
    }

    var dot =
        badge.querySelector(
            ".status-dot"
        );

    if (dot)
    {
        dot.classList.remove(
            "waiting",
            "online",
            "offline"
        );

        dot.classList.add(state);
    }
}

function formatNumber(value, digits)
{
    var number =
        Number(value);

    if (!Number.isFinite(number))
    {
        return "--";
    }

    return number.toFixed(digits);
}

function dataValid(data)
{
    return data &&
           (
               data.valid === true ||
               data.valid === 1
           );
}

async function requestJSON(
    url,
    options
)
{
    var requestOptions =
        options || {};

    requestOptions.cache =
        "no-store";

    var separator =
        url.indexOf("?") >= 0
            ? "&"
            : "?";

    var response =
        await fetch(
            url +
            separator +
            "_=" +
            Date.now(),
            requestOptions
        );

    if (!response.ok)
    {
        throw new Error(
            "HTTP " +
            response.status +
            " for " +
            url
        );
    }

    return response.json();
}

window.addEventListener(
    "DOMContentLoaded",
    function()
    {
        getUpdateStatus();

        getLocalTime();

        startLocalTimeInterval();

        getConnectInfo();

        getAPSSID();

        getSensorReadings();

        startSensorInterval();

        var connectButton =
            byId("connect_wifi");

        var disconnectButton =
            byId("disconnect_wifi");

        if (connectButton)
        {
            connectButton.addEventListener(
                "click",
                checkCredentials
            );
        }

        if (disconnectButton)
        {
            disconnectButton.addEventListener(
                "click",
                disconnectWifi
            );
        }
    }
);

function getFileInfo()
{
    var input =
        byId("selected_file");

    if (!input ||
        !input.files ||
        input.files.length === 0)
    {
        setText(
            "file_info",
            ""
        );

        return;
    }

    var file =
        input.files[0];

    setText(
        "file_info",
        "File: " +
        file.name +
        " • " +
        file.size +
        " bytes"
    );
}

function updateFirmware()
{
    var fileSelect =
        byId("selected_file");

    if (!fileSelect ||
        !fileSelect.files ||
        fileSelect.files.length !== 1)
    {
        window.alert(
            "Select a firmware file first"
        );

        return;
    }

    var file =
        fileSelect.files[0];

    var formData =
        new FormData();

    formData.set(
        "file",
        file,
        file.name
    );

    setText(
        "ota_update_status",
        "Uploading " +
        file.name +
        " — firmware update in progress..."
    );

    var request =
        new XMLHttpRequest();

    request.upload.addEventListener(
        "progress",
        function(event)
        {
            if (event.lengthComputable)
            {
                getUpdateStatus();
            }
        }
    );

    request.open(
        "POST",
        API.otaUpdate
    );

    request.responseType =
        "blob";

    request.send(
        formData
    );
}

async function getUpdateStatus()
{
    try
    {
        var data =
            await requestJSON(
                API.otaStatus,
                {
                    method: "POST",

                    headers: {
                        "Content-Type":
                            "text/plain"
                    },

                    body:
                        "ota_update_status"
                }
            );

        if (data.compile_date ||
            data.compile_time)
        {
            setText(
                "latest_firmware",

                (data.compile_date || "") +
                " — " +
                (data.compile_time || "")
            );
        }

        if (data.ota_update_status === 1)
        {
            seconds = 10;

            otaRebootTimer();
        }
        else if (
            data.ota_update_status === -1
        )
        {
            setHTML(
                "ota_update_status",
                "<span class='rd'>" +
                "Upload failed" +
                "</span>"
            );
        }
    }
    catch (error)
    {
        console.warn(
            "OTA status request failed:",
            error
        );
    }
}

function otaRebootTimer()
{
    setText(
        "ota_update_status",
        "OTA complete. Rebooting in " +
        seconds +
        " s"
    );

    seconds--;

    if (seconds <= 0)
    {
        clearTimeout(
            otaTimerVar
        );

        window.location.reload();

        return;
    }

    otaTimerVar =
        setTimeout(
            otaRebootTimer,
            1000
        );
}

function stopWifiConnectStatusInterval()
{
    if (wifiConnectInterval !== null)
    {
        clearInterval(
            wifiConnectInterval
        );

        wifiConnectInterval =
            null;
    }
}

async function getWifiConnectStatus()
{
    try
    {
        var data =
            await requestJSON(
                API.wifiConnectStatus,
                {
                    method: "POST",

                    headers: {
                        "Content-Type":
                            "text/plain"
                    },

                    body:
                        "wifi_connect_status"
                }
            );

        setText(
            "wifi_connect_status",
            "Connecting..."
        );

        if (data.wifi_connect_status === 2)
        {
            setHTML(
                "wifi_connect_status",
                "<span class='rd'>" +
                "Connection failed. " +
                "Check AP credentials." +
                "</span>"
            );

            stopWifiConnectStatusInterval();
        }
        else if (
            data.wifi_connect_status === 3
        )
        {
            setHTML(
                "wifi_connect_status",
                "<span class='gr'>" +
                "Connection established" +
                "</span>"
            );

            stopWifiConnectStatusInterval();

            getConnectInfo();
        }
    }
    catch (error)
    {
        console.warn(
            "Wi-Fi status request failed:",
            error
        );
    }
}

function startWifiConnectStatusInterval()
{
    stopWifiConnectStatusInterval();

    wifiConnectInterval =
        setInterval(
            getWifiConnectStatus,
            2800
        );
}

async function connectWifi()
{
    var ssidInput =
        byId("connect_ssid");

    var passwordInput =
        byId("connect_pass");

    var ssid =
        ssidInput
            ? ssidInput.value
            : "";

    var password =
        passwordInput
            ? passwordInput.value
            : "";

    try
    {
        var response =
            await fetch(
                API.wifiConnect +
                "?_=" +
                Date.now(),
                {
                    method: "POST",
                    cache: "no-store",

                    headers: {
                        "my-connect-ssid": ssid,
                        "my-connect-pwd": password
                    }
                }
            );

        if (!response.ok)
        {
            throw new Error(
                "HTTP " +
                response.status
            );
        }

        var result =
            await response.json();

        if (!result.accepted)
        {
            throw new Error(
                "ESP32 rejected credentials"
            );
        }

        setText(
            "wifi_connect_status",
            "Credentials accepted. Connecting..."
        );

        startWifiConnectStatusInterval();
    }
    catch (error)
    {
        setHTML(
            "wifi_connect_status",
            "<span class='rd'>" +
            "Unable to send Wi-Fi connection request." +
            "</span>"
        );

        console.error(
            "Wi-Fi connect request failed:",
            error
        );
    }
}


function checkCredentials()
{
    var ssidInput =
        byId("connect_ssid");

    var passwordInput =
        byId("connect_pass");

    var ssid =
        ssidInput
            ? ssidInput.value.trim()
            : "";

    var password =
        passwordInput
            ? passwordInput.value
            : "";

    var errors = [];

    if (ssid === "")
    {
        errors.push(
            "SSID cannot be empty"
        );
    }

    if (password === "")
    {
        errors.push(
            "Password cannot be empty"
        );
    }

    if (errors.length > 0)
    {
        setHTML(
            "wifi_connect_credentials_errors",

            "<span class='rd'>" +
            errors.join(" • ") +
            "</span>"
        );

        return;
    }

    setText(
        "wifi_connect_credentials_errors",
        ""
    );

    connectWifi();
}

function showPassword()
{
    var input =
        byId("connect_pass");

    if (input)
    {
        input.type =
            input.type === "password"
                ? "text"
                : "password";
    }
}

async function getConnectInfo()
{
    try
    {
        var data =
            await requestJSON(
                API.wifiConnectInfo
            );

        if (data.ap)
        {
            setText(
                "connected_ap",
                data.ap
            );

            setText(
                "wifi_connect_ip",
                data.ip || "--"
            );

            setText(
                "wifi_connect_netmask",
                data.netmask || "--"
            );

            setText(
                "wifi_connect_gw",
                data.gw || "--"
            );

            var button =
                byId("disconnect_wifi");

            if (button)
            {
                button.style.display =
                    "inline-block";
            }
        }
    }
    catch (error)
    {
        console.warn(
            "Wi-Fi info request failed:",
            error
        );
    }
}




async function disconnectWifi()
{
    setText(
        "wifi_connect_status",
        "Disconnecting..."
    );

    try
    {
        var response =
            await fetch(
                API.wifiDisconnect +
                "?_=" +
                Date.now(),
                {
                    method: "DELETE",
                    cache: "no-store"
                }
            );

        if (!response.ok)
        {
            throw new Error(
                "HTTP " +
                response.status
            );
        }

        var result =
            await response.json();

        if (!result.disconnected)
        {
            throw new Error(
                "Disconnect request rejected"
            );
        }

        setText(
            "wifi_connect_status",
            "Disconnected"
        );

        setText(
            "connected_ap",
            "--"
        );

        setText(
            "wifi_connect_ip",
            "--"
        );

        setText(
            "wifi_connect_netmask",
            "--"
        );

        setText(
            "wifi_connect_gw",
            "--"
        );

        var disconnectButton =
            byId("disconnect_wifi");

        if (disconnectButton)
        {
            disconnectButton.style.display =
                "none";
        }

        setTimeout(
            function()
            {
                window.location.reload();
            },
            1500
        );
    }
    catch (error)
    {
        setHTML(
            "wifi_connect_status",
            "<span class='rd'>" +
            "Unable to disconnect Wi-Fi." +
            "</span>"
        );

        console.error(
            "Wi-Fi disconnect failed:",
            error
        );
    }
}



function startLocalTimeInterval()
{
    setInterval(
        getLocalTime,
        10000
    );
}

async function getLocalTime()
{
    try
    {
        var data =
            await requestJSON(
                API.localTime
            );

        if (data.time)
        {
            setText(
                "local_time",
                data.time
            );
        }
    }
    catch (error)
    {
        console.warn(
            "Local time request failed:",
            error
        );
    }
}

async function getAPSSID()
{
    try
    {
        var data =
            await requestJSON(
                API.apSSID
            );

        if (data.ssid)
        {
            setText(
                "ap_ssid",
                data.ssid
            );
        }
    }
    catch (error)
    {
        console.warn(
            "AP SSID request failed:",
            error
        );
    }
}

function startSensorInterval()
{
    if (sensorInterval !== null)
    {
        clearInterval(
            sensorInterval
        );
    }

    sensorInterval =
        setInterval(
            getSensorReadings,
            2000
        );
}

async function getSensorReadings()
{
    if (sensorPollBusy)
    {
        return;
    }

    sensorPollBusy = true;

    try
    {
        await Promise.allSettled(
            [
                getCCS811SensorReadings(),
                getBME280SensorReadings(),
                getSTM32Telemetry()
            ]
        );
    }
    finally
    {
        sensorPollBusy = false;
    }
}

async function getCCS811SensorReadings()
{
    try
    {
        var data =
            await requestJSON(
                API.ccs811Sensor
            );

        if (!dataValid(data))
        {
            setPill(
                "ccs811_status",
                "waiting",
                "Waiting"
            );

            return;
        }

        setText(
            "ccs811_eco2",
            data.eco2 !== undefined
                ? data.eco2
                : "--"
        );

        setText(
            "ccs811_tvoc",
            data.tvoc !== undefined
                ? data.tvoc
                : "--"
        );

        setText(
            "ccs811_current",
            data.raw_current !== undefined
                ? data.raw_current
                : "--"
        );

        setText(
            "ccs811_adc",
            data.raw_voltage_adc !== undefined
                ? data.raw_voltage_adc
                : "--"
        );

        setText(
            "ccs811_voltage",
            data.raw_voltage !== undefined
                ? data.raw_voltage
                : "--"
        );

        setPill(
            "ccs811_status",
            "online",
            "Online"
        );
    }
    catch (error)
    {
        setPill(
            "ccs811_status",
            "offline",
            "Read failed"
        );

        console.warn(
            "CCS811 request failed:",
            error
        );
    }
}

async function getBME280SensorReadings()
{
    try
    {
        var data =
            await requestJSON(
                API.bme280Sensor
            );

        if (!dataValid(data))
        {
            setPill(
                "bme280_status",
                "waiting",
                "Waiting"
            );

            return;
        }

        setText(
            "bme280_sensor_type",
            data.sensor || "--"
        );

        setText(
            "bme280_chip_id",
            data.chip_id !== undefined
                ? data.chip_id
                : "--"
        );

        setText(
            "bme280_temperature",
            formatNumber(
                data.temperature,
                2
            )
        );

        setText(
            "bme280_pressure_pa",
            formatNumber(
                data.pressure_pa,
                2
            )
        );

        setText(
            "bme280_pressure_hpa",
            formatNumber(
                data.pressure_hpa,
                2
            )
        );

        if (data.humidity_available === true ||
            data.humidity_available === 1)
        {
            setText(
                "bme280_humidity",
                formatNumber(
                    data.humidity,
                    2
                )
            );
        }
        else
        {
            setText(
                "bme280_humidity",
                "N/A"
            );
        }

        setPill(
            "bme280_status",
            "online",
            "Online"
        );
    }
    catch (error)
    {
        setPill(
            "bme280_status",
            "offline",
            "Read failed"
        );

        console.warn(
            "BME280 request failed:",
            error
        );
    }
}

function markSTM32Cards(
    state,
    text
)
{
    [
        "bme680_status",
        "max31865_status",
        "ds18b20_status",
        "pm25_status",
        "mpu9250_status"
    ].forEach(
        function(id)
        {
            setPill(
                id,
                state,
                text
            );
        }
    );
}

async function getSTM32Telemetry()
{
    try
    {
        var data =
            await requestJSON(
                API.stm32Telemetry
            );

        if (!dataValid(data))
        {
            markSTM32Cards(
                "waiting",
                "Waiting"
            );

            setSecureLinkState(
                "waiting",
                "WAITING"
            );

            setText(
                "stm32_sequence",
                "--"
            );

            return;
        }

        markSTM32Cards(
            "online",
            "Authenticated"
        );

        setSecureLinkState(
            "online",
            "AUTHENTICATED"
        );

        setText(
            "stm32_sequence",
            data.sequence !== undefined
                ? data.sequence
                : "--"
        );

        var bme680 =
            data.bme680 || {};

        setText(
            "bme680_temperature",
            formatNumber(
                bme680.temperature,
                2
            )
        );

        setText(
            "bme680_pressure",
            formatNumber(
                bme680.pressure,
                2
            )
        );

        setText(
            "bme680_humidity",
            formatNumber(
                bme680.humidity,
                2
            )
        );

        setText(
            "bme680_iaq_score",
            bme680.iaq_score !== undefined
                ? bme680.iaq_score
                : "--"
        );

        setText(
            "bme680_iaq_label",
            bme680.iaq_label || "--"
        );

        var max31865 =
            data.max31865 || {};

        setText(
            "max31865_temperature",
            formatNumber(
                max31865.temperature,
                3
            )
        );

        var ds18b20 =
            data.ds18b20 || {};

        var temperatures =
            Array.isArray(
                ds18b20.temperature
            )
                ? ds18b20.temperature
                : [];

        for (var i = 0; i < 5; i++)
        {
            setText(
                "ds18b20_t" + i,

                temperatures[i] !== undefined
                    ? formatNumber(
                        temperatures[i],
                        3
                    )
                    : "--"
            );
        }

        var pm =
            data.pm25 || {};

        setText(
            "pm1_std",
            pm.pm1_0_standard !== undefined
                ? pm.pm1_0_standard
                : "--"
        );

        setText(
            "pm25_std",
            pm.pm2_5_standard !== undefined
                ? pm.pm2_5_standard
                : "--"
        );

        setText(
            "pm10_std",
            pm.pm10_standard !== undefined
                ? pm.pm10_standard
                : "--"
        );

        setText(
            "pm1_atm",
            pm.pm1_0_atmospheric !== undefined
                ? pm.pm1_0_atmospheric
                : "--"
        );

        setText(
            "pm25_atm",
            pm.pm2_5_atmospheric !== undefined
                ? pm.pm2_5_atmospheric
                : "--"
        );

        setText(
            "pm10_atm",
            pm.pm10_atmospheric !== undefined
                ? pm.pm10_atmospheric
                : "--"
        );

        setText(
            "p03",
            pm.particles_0_3um !== undefined
                ? pm.particles_0_3um
                : "--"
        );

        setText(
            "p05",
            pm.particles_0_5um !== undefined
                ? pm.particles_0_5um
                : "--"
        );

        setText(
            "p10",
            pm.particles_1_0um !== undefined
                ? pm.particles_1_0um
                : "--"
        );

        setText(
            "p25",
            pm.particles_2_5um !== undefined
                ? pm.particles_2_5um
                : "--"
        );

        setText(
            "p50",
            pm.particles_5_0um !== undefined
                ? pm.particles_5_0um
                : "--"
        );

        setText(
            "p100",
            pm.particles_10um !== undefined
                ? pm.particles_10um
                : "--"
        );

        var imu =
            data.mpu9250 || {};

        setText(
            "mpu_ax",
            formatNumber(
                imu.ax,
                3
            )
        );

        setText(
            "mpu_ay",
            formatNumber(
                imu.ay,
                3
            )
        );

        setText(
            "mpu_az",
            formatNumber(
                imu.az,
                3
            )
        );

        setText(
            "mpu_gx",
            formatNumber(
                imu.gx,
                3
            )
        );

        setText(
            "mpu_gy",
            formatNumber(
                imu.gy,
                3
            )
        );

        setText(
            "mpu_gz",
            formatNumber(
                imu.gz,
                3
            )
        );
    }
    catch (error)
    {
        markSTM32Cards(
            "offline",
            "Unavailable"
        );

        setSecureLinkState(
            "offline",
            "UNAVAILABLE"
        );

        console.warn(
            "STM32 telemetry request failed:",
            error
        );
    }
}