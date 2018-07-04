/*battery_regs struct arry*/
struct battery_regs
{
	unsigned char reg;
	unsigned char status;
	int errno;
	char *desc;
	unsigned long value;
};
struct battery_regs batregs[] = {
	{0x02, 0, 0, "At rate", 0},
	{0x04, 0, 0, "At rate time to empty", 0},
	{0x06, 0, 0, "Temperature", 0},
	{0x08, 0, 0, "voltage", 0},
	{0x0C, 0, 0, "Normal avaliable capacity", 0},
	{0x0E, 0, 0, "Full avaliable capacity", 0},
	{0x10, 0, 0, "Remaining capacity", 0},
	{0x12, 0, 0, "Full charge capacity", 0},
	{0x14, 0, 0, "Average current", 0},
	{0x16, 0, 0, "Run time to empty", 0},
	{0x28, 0, 0, "Internal temperature", 0},
	{0x2a, 0, 0, "One dsg cyclecount", 0},
	{0x2c, 0, 0, "Reative state of charge", 0},
	{0x2e, 0, 0, "State of health", 0},
	{0x30, 0, 0, "Current", 0},
	{0x32, 0, 0, "Safety status", 0},
	{0x70, 0, 0, "Pack voltage", 0},
};

/*bms' cmds*/
const static struct _cmd_def{
	char* name;
	void (*proc) (struct cmdline*);
	char* help_msg;
} cmds[] = {
	{"help", cmd_help, "show this message"},
	{"reset", cmd_reset, "reset cpu"},
	{"led", cmd_led, "play led pattern"},
	{"bat", cmd_battery, "comm with battery"},
	{"12v", cmd_12v, "detect 12v on"},
	{"lock", cmd_lock, "car lock ctrl"},
};
