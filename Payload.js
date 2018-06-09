function Decoder(bytes, port) {
var mbar = 970+((bytes[1] >> 2) & 0x3F);
var temperature = -2400+6.25*(((bytes[1] & 0x03) << 8) | bytes[0]); 
var humidity = 5+3*((bytes[3] >> 3) & 0x1F);
var co2 = 4*(((bytes[3] & 0x07) << 8) | bytes[2]);
var tvoc = 5*bytes[4];
return {
mbar: mbar,
celcius: temperature / 100.0,
humidity:humidity,
co2:co2,
tvoc:tvoc
};
}
