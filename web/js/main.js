function makeRequest(url, data, cb, requestType)
{
    var xmlhttp = new XMLHttpRequest();

    xmlhttp.onreadystatechange = function() {
        if (xmlhttp.readyState == 4) {
            if (xmlhttp.status==0 || xmlhttp.status>=400)
                return;

            cb(xmlhttp.responseText);
        }
    };

    if (typeof requestType === 'undefined')
        requestType = "POST";

    xmlhttp.open(requestType, url, true);
    xmlhttp.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
    xmlhttp.send(data);
}

function getParams(url)
{
    makeRequest(url, null, finishGettingParam, "GET");
}

function finishGettingParam(response)
{
    try {
        var params = JSON.parse(response);
        for (var paramId in params)
            writeParam(params[paramId], paramId);
        setTimeout(getParams,5000,"/api/sensordata");
    } catch(e) {return;}
}

function writeParam(paramVal,paramId)
{
    if (paramId == "set_temp" || paramId == "set_humid")
        document.getElementById(paramId).value = paramVal;
    else
        document.getElementById(paramId).innerHTML = paramVal;
}

function setParam(url, value, param)
{
    var data = {};
    data[param] = value;
    makeRequest(url, data, finishSettingParam);
}

function finishSettingParam()
{
    try {
        var res = JSON.parse(response);
        // warning
    } catch(e) {return;}
}

function changeTab(tableId)
{
    var tabs = ["home","outside","settings"];
    for (var tab in tabs) {
        if (tabs[tab] == tableId) {
            document.getElementById("hvac_"+tableId).style.display = "table";//"block";
            document.getElementById(tableId).style.filter = "none";
            if (tableId=="home")
                document.getElementById("hvac_status_tbl").style.display = "table";
            else
                document.getElementById("hvac_status_tbl").style.display = "none";
        } else {
            document.getElementById("hvac_"+tabs[tab]).style.display = "none";
            document.getElementById(tabs[tab]).style.filter = "grayscale(100%)";
        }
    }
}

setTimeout(getParams, 1000, "/api/sensordata");
