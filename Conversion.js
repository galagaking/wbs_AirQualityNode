var pres = msg.payload.pressure;
var temp = msg.payload.temperature;
var hum = msg.payload.humidity;
var tco2=msg.payload.co2;
var ttvoc=msg.payload.tvoc;

if (ttvoc > 1000) {
    var tservo=170; }
else {
    var tservo=10; }
if (temp > 25) {
    var tled=1; }
else {
    var tled=0;
}
msg.payload = {
  pressure: pres.toFixed(0),
  temp : temp.toFixed(2),
//  humidity: hum.toFixed(0),
  tvoc: ttvoc.toFixed(0),
  co2:tco2.toFixed(0),
  led:tled,
  servo:tservo
};
return msg;
