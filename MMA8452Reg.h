// http://www.freescale.com/files/sensors/doc/data_sheet/MMA8452Q.pdf
// Pages 18-19

#define STATUS				0x07	// Real time status
#define OUT_X_MSB			0x01	// [7:0] are 8 MSBs of 12-bit sample
#define OUT_X_LSB			0x02	// [7:4] are 4 LSBs of 12-bit sample
#define OUT_Y_MSB			0x03	// [7:0] are 8 MSBs of 12-bit sample
#define OUT_Y_LSB			0x04	// [7:4] are 4 LSBs of 12-bit sample
#define OUT_Z_MSB			0x05	// [7:0] are 8 MSBs of 12-bit sample
#define OUT_Z_LSB			0x06	// [7:4] are 4 LSBs of 12-bit sample
// Reserved 0x07-0x08
#define SYSMOD				0x0B	// Current System Mode
#define INT_SOURCE			0x0C	// Interrupt status
#define WHO_AM_I			0x0D	// Device ID (0x2A)
#define XYZ_DATA_CFG		0x0E	// HPF Data Out and Dynamic Range Settings
#define HP_FILTER_CUTOFF	0x0F	// Cutoff frequency is set to 16 Hz @ 800 Hz
#define PL_STATUS			0x10	// Landscape/Portrait orientation status
#define PL_CFG				0x11	// Landscape/Portrait configuration
#define PL_COUNT			0x12	// Landscape/Portrait debounce counter
#define PL_BF_ZCOMP			0x13	// Back-Front, Z-Lock Trip threshold
#define P_L_THS_REG			0x14	// Portrait to Landscape Trip Angle is 29°
#define FF_MT_CFG			0x15	// Freefall/Motion functional block configuration
#define FF_MT_SRC			0x16	// Freefall/Motion event source register
#define FF_MT_THS			0x17	// Freefall/Motion threshold register
#define FF_MT_COUNT			0x18	// Freefall/Motion debounce counter
// Reserved 0x19-0x1C
#define TRANSIENT_CFG		0x1D	// Transient functional block configuration
#define TRANSIENT_SRC		0x1E	// Transient event status register
#define TRANSIENT_THS		0x1F	// Transient event threshold
#define TRANSIENT_COUNT		0x20	// Transient debounce counter
#define PULSE_CFG			0x21	// ELE, Double_XYZ or Single_XYZ
#define PULSE_SRC			0x22	// EA, Double_XYZ or Single_XYZ
#define PULSE_THSX			0x23	// X pulse threshold
#define PULSE_THSY			0x24	// Y pulse threshold
#define PULSE_THSZ			0x25	// Z pulse threshold
#define PULSE_TMLT			0x26	// Time limit for pulse
#define PULSE_LTCY			0x27	// Latency time for 2nd pulse
#define PULSE_WIND			0x28	// Window time for 2nd pulse
#define ASLP_COUNT			0x29	// Counter setting for Auto-SLEEP
#define CTRL_REG1			0x2A	// Data Rate, ACTIVE Mode
#define CTRL_REG2			0x2B	// Sleep Enable, OS Modes, RST, ST
#define CTRL_REG3			0x2C	// Wake from Sleep, IPOL, PP_OD
#define CTRL_REG4			0x2D	// Interrupt enable register
#define CTRL_REG5			0x2E	// Interrupt pin (INT1/INT2) map
#define OFF_X				0x2F	// X-axis offset adjust
#define OFF_Y				0x30	// Y-axis offset adjust
#define OFF_Z				0x31	// Z-axis offset adjust
// Reserved 0x40-0x7F