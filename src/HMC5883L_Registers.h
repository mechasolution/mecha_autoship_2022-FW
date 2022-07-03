//Address map for registers
#define configA 0x00
#define configB 0x01
#define mode 0x02
#define dataOutX_U 0x03
#define dataOutX_L 0x04
// #define dataOutZ_L 0x05
#define dataOutZ_L 0x06
// #define dataOutY_L 0x07
#define dataOutY_L 0x08
#define statusReg 0x09
//Operating modes sent to Mode register (0x02)
#define continuous 0x00
#define single 0x01
#define idle 0x02
#define i2c_addr 0x1E
#define gain 1090