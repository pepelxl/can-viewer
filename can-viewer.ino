#include <SPI.h>
#include <mcp2515.h>
//Пин, к которому подключен пьезодинамик.
int piezoPin = 3;
// раскомментировать для отладки
#define DEBUG // для отладки на столе. переводит контроллер в LoopbackMode и включает терминал
#define CS_PIN_CAN0 9  //chip select pin
#define CAN_SPEED CAN_125KBPS // салонная fault tolerant
#define CAN_FREQ MCP_16MHZ // кварц mcp2515

uint32_t id_target = 0x350; // посылка, которую будем парсить

// дефайн перед подключением либы - использовать microWire (лёгкая либа для I2C)
#define USE_MICRO_WIRE
// дефайн перед подключением либы - скорость SPI
#define OLED_SPI_SPEED 4000000ul
#include <GyverOLED.h> 
#include <icons_7x7.h>
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
MCP2515 mcp2515(CS_PIN_CAN0); // CAN-BUS Shield

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c %c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

struct can_frame frame; // прочитанная посылка
__u8 old[8]{ 0, 0, 0, 0, 0, 0, 0, 0 }; // данные для сравнения
bool readOK = false;

// инициализация mcp2515
void resetMcp2515()
{
	mcp2515.reset(); // после сброса чип находится в режиме конфигурации
	while (mcp2515.setBitrate(CAN_SPEED, CAN_FREQ) != MCP2515::ERROR_OK) delay(100);  // устанавливаем скорость
	// задаём фильтры для приёма одного сообщения.
	while (mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF) != MCP2515::ERROR_OK) delay(100);
	while (mcp2515.setFilter(MCP2515::RXF0, false, id_target) != MCP2515::ERROR_OK) delay(100);
	while (mcp2515.setFilter(MCP2515::RXF1, false, id_target) != MCP2515::ERROR_OK) delay(100);
	while (mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF) != MCP2515::ERROR_OK) delay(100);
	while (mcp2515.setFilter(MCP2515::RXF2, false, id_target) != MCP2515::ERROR_OK) delay(100);
	while (mcp2515.setFilter(MCP2515::RXF3, false, id_target) != MCP2515::ERROR_OK) delay(100);
	while (mcp2515.setFilter(MCP2515::RXF4, false, id_target) != MCP2515::ERROR_OK) delay(100);
	while (mcp2515.setFilter(MCP2515::RXF5, false, id_target) != MCP2515::ERROR_OK) delay(100);
#if defined DEBUG
	while (mcp2515.setLoopbackMode() != MCP2515::ERROR_OK) delay(100);
#else
	while (mcp2515.setListenOnlyMode() != MCP2515::ERROR_OK) delay(100); // включаем mcp2515
#endif
}

void printErrorInt(uint8_t interrupt)
{
	uint8_t stroka = 0;
	oled.clear();
	oled.setCursor(0, stroka++);
	oled.print("ERROR in interrupt");
	if (interrupt & MCP2515::CANINTF_RX0IF)
	{
		oled.setCursor(0, stroka++);
		oled.print("CANINTF_RX0IF");
	}
	if (interrupt & MCP2515::CANINTF_RX1IF)
	{
		oled.setCursor(0, stroka++);
		oled.print("CANINTF_RX1IF");
	}
	if (interrupt & MCP2515::CANINTF_TX0IF)
	{
		oled.setCursor(0, stroka++);
		oled.print("CANINTF_TX0IF");
	}
	if (interrupt & MCP2515::CANINTF_TX1IF)
	{
		oled.setCursor(0, stroka++);
		oled.print("CANINTF_TX1IF");
	}
	if (interrupt & MCP2515::CANINTF_TX2IF)
	{
		oled.setCursor(0, stroka++);
		oled.print("CANINTF_TX2IF");
	}
	if (interrupt & MCP2515::CANINTF_ERRIF)
	{
		oled.setCursor(0, stroka++);
		oled.print("CANINTF_ERRIF");
	}
	if (interrupt & MCP2515::CANINTF_WAKIF)
	{
		oled.setCursor(0, stroka++);
		oled.print("CANINTF_WAKIF");
	}
	if (interrupt & MCP2515::CANINTF_MERRF)
	{
		oled.setCursor(0, stroka++);
		oled.print("CANINTF_MERRF");
	}
	delay(2000);
}

void readMcp2515()
{
	uint8_t interrupt = mcp2515.getInterrupts() & 0xE3;
	if ((interrupt | MCP2515::CANINTF_RX0IF | MCP2515::CANINTF_RX1IF) != 0x3U)
	{
		printErrorInt(interrupt);
		resetMcp2515();
		return;
	}
	if (interrupt & MCP2515::CANINTF_RX0IF)
	{
		if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) readOK = true;
	}
	if (interrupt & MCP2515::CANINTF_RX1IF)
	{
		if (mcp2515.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK) readOK = true;
	}
}


void setup() {
#if defined DEBUG
	Serial.begin(115200);
#endif
#pragma region hello 
	oled.init();
	oled.clear();
	oled.setCursor(0, 0);
	oled.print("pepelxl v0.1 viewer");
	oled.setCursor(0, 1);
	delay(2000);
#pragma endregion инициализируем дисплей и выводим приветственное сообщение

	resetMcp2515(); // инициализируем can контроллер
	attachInterrupt(0, readMcp2515, FALLING); // устанавливаем функцию для прерывания.
}

void runfind(const char* str = "")
{
	tone(piezoPin, 1000, 5000);
	oled.clear(11, 0, 127, 7);
	oled.setScale(4);
	oled.setCursor(18, 2);
	oled.print("NEW");
	oled.setScale(1);
	oled.setCursor(18, 7);
	oled.print(str);
}


void loop() {
#if defined DEBUG
	char rc;
	if (Serial.available() > 0) {
		delay(10);
		rc = Serial.read();
		struct can_frame s;
		if (rc == '0')
		{
			s.can_id = 0x350;
			s.can_dlc = 7;
			s.data[0] = 0x00;
			s.data[1] = 0xff;
			s.data[2] = 0xff;
			s.data[3] = 0x48;
			s.data[4] = 0x0d;
			s.data[5] = 0x80;
			s.data[6] = 0x04;
			mcp2515.sendMessage(&s);
		}
		if (rc == '1')
		{
			s.can_id = 0x350;
			s.can_dlc = 7;
			s.data[0] = 0x14;
			s.data[1] = 0xff;
			s.data[2] = 0xff;
			s.data[3] = 0x48;
			s.data[4] = 0x0d;
			s.data[5] = 0x99;
			s.data[6] = 0x14;
			mcp2515.sendMessage(&s);
		}
		else if (rc == '2')
		{
			s.can_id = 0x350;
			s.can_dlc = 7;
			s.data[0] = 0x20;
			s.data[1] = 0xff;
			s.data[2] = 0xff;
			s.data[3] = 0x00;
			s.data[4] = 0x0d;
			s.data[5] = 0xB0;
			s.data[6] = 0x24;
			mcp2515.sendMessage(&s);
		}
		else if (rc == '3')
		{
			s.can_id = 0x350;
			s.can_dlc = 7;
			s.data[0] = 0x00;
			s.data[1] = 0xff;
			s.data[2] = 0xff;
			s.data[3] = 0x0d;
			s.data[4] = 0x00;
			s.data[5] = 0x81;
			s.data[6] = 0x34;
			mcp2515.sendMessage(&s);
		}
		if (rc == '4')
		{
			s.can_id = 0x350;
			s.can_dlc = 7;
			s.data[0] = 0x00;
			s.data[1] = 0xff;
			s.data[2] = 0xff;
			s.data[3] = 0x48;
			s.data[4] = 0x0d;
			s.data[5] = 0x89;
			s.data[6] = 0x44;
			mcp2515.sendMessage(&s);
		}
		else if (rc == '5')
		{
			s.can_id = 0x350;
			s.can_dlc = 7;
			s.data[0] = 0x20;
			s.data[1] = 0xff;
			s.data[2] = 0xff;
			s.data[3] = 0x00;
			s.data[4] = 0x0d;
			s.data[5] = 0xB0;
			s.data[6] = 0x54;
			mcp2515.sendMessage(&s);
		}
		else if (rc == '6')
		{
			s.can_id = 0x350;
			s.can_dlc = 7;
			s.data[0] = 0x00;
			s.data[1] = 0xff;
			s.data[2] = 0xff;
			s.data[3] = 0x0d;
			s.data[4] = 0x00;
			s.data[5] = 0x81;
			s.data[6] = 0x64;
			mcp2515.sendMessage(&s);
		}
		if (rc == '7')
		{
			s.can_id = 0x350;
			s.can_dlc = 7;
			s.data[0] = 0x00;
			s.data[1] = 0xff;
			s.data[2] = 0xff;
			s.data[3] = 0x48;
			s.data[4] = 0x0d;
			s.data[5] = 0x89;
			s.data[6] = 0x74;
			mcp2515.sendMessage(&s);
		}
		else if (rc == '8')
		{
			s.can_id = 0x350;
			s.can_dlc = 7;
			s.data[0] = 0x00;
			s.data[1] = 0xff;
			s.data[2] = 0xff;
			s.data[3] = 0x00;
			s.data[4] = 0x0d;
			s.data[5] = 0xB0;
			s.data[6] = 0x84;
			mcp2515.sendMessage(&s);
		}
		else if (rc == '9')
		{
			s.can_id = 0x350;
			s.can_dlc = 7;
			s.data[0] = 0x00;
			s.data[1] = 0xff;
			s.data[2] = 0xff;
			s.data[3] = 0x0d;
			s.data[4] = 0x00;
			s.data[5] = 0x81;
			s.data[6] = 0x94;
			mcp2515.sendMessage(&s);
		}
	}
#endif
	if (readOK)
	{
		bool compare = true;
		for (uint8_t i = 0; i < frame.can_dlc; i++)
		{
			if (old[i] != frame.data[i]) { compare = false; break; }
		}
		if (compare) return;
		for (uint8_t i = 0; i < frame.can_dlc; i++)
		{
			old[i] = frame.data[i];
		}
		oled.clear();
		for (uint8_t i = 0; i < frame.can_dlc; i++)
		{
			oled.setCursor(0, i);
			char b[2];
			sprintf(b, "%02X", old[i]);
			oled.print(b);
		}
		if (frame.can_id == 0x350) prIDx350();
		else
		{
			for (uint8_t i = 0; i < frame.can_dlc; i++)
			{
				oled.setCursor(18, i);
				char b[9];
				sprintf(b, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(old[i]));
				oled.print(b);
			}
		}
		readOK = false;
	}



}

void drawIcon7x7(byte index) {
	size_t s = sizeof icons_7x7[index];//можно так, а можно просто 7 
	for (unsigned int i = 0; i < s; i++) {
		oled.drawByte(pgm_read_byte(&(icons_7x7[index][i])));
	}
}

const char* temp_climat[]{ "LO","14","15","16","17","18","18,5","19","19,5","20","20,5","21","21,5","22","22,5","23","23,5","24","25","26","27","28","HI" };
void prIDx350() //climate status (индикация для отображения на магнитоле\приборки\панели климата)
{
	/*
	 D0
	 1100 1000 не известно
	 0010 0000 индикация обогрева заднего стекла. Появляется один раз при нажатии кнопки(включение\выключение). назначение не понятно.
	 0001 0100 индикация обогрева переднего стекла. Активация устанавливает сразу четыре бита D0[14] D5[10] D6[10]
	 0000 0011 индикация интенсивности вентиляции Мягкий\Норма\Быстро
	 */
	oled.setCursor(18, 0);
	if (!(old[0] & 0x08)) oled.print("AC"); // индикация выключено кондиционера
	oled.setCursor(36, 0);
	uint8_t intensive = old[0] & 0x3; // индикация интенсивности вентиляции Мягкий\Норма\Быстро
	if (intensive == 0) oled.print("soft");
	else if (intensive == 1) oled.print("medium");
	else if (intensive == 2) oled.print("fast");
	else { runfind("D0 intensive"); return; }
	if (old[0] & 0x10)
	{
		if ((old[0] & 0x10) && (old[0] & 0x04) && (old[5] & 0x10) && ((old[6] >> 4) == 1)) { oled.print(" F"); } // индикация обогрев лобового стекла
		else { runfind("обдув стекла"); return; }
	}
	if (old[0] & 0x20) oled.print(" R"); // индикация обогрев заднего стекла
	if (old[0] & 0xC0) { runfind("искл. D0"); return; }

	/*
	D1 - не известно, всегда 0xFF
	*/
	if (old[1] != 0xFF) { runfind("искл. D1"); return; }

	/*
	D2 - не известно, всегда 0xFF
	*/
	if (old[2] != 0xFF) { runfind("искл. D2"); return; }

	/*
	D3
	0001 1111 температура слева
	0100 0000 статус mono, климат меняет температуру на обоих ползунках с левой крутилки
	1010 0000 не известно
	*/
	oled.setCursor(18, 3);
	if (old[3] & 0x40) oled.print("Mono  T: "); // индикация mono режима
	else oled.print("Left  T: ");
	uint8_t tl = old[3] & 0x1F;
	if (tl < 23) oled.print(temp_climat[tl]); // индикация температуры слева
	else { runfind("D3 темп"); return; }
	if (old[3] & 0xA0) { runfind("искл. D3"); return; }

	/*
	D4
	0001 1111 температура справа
	0100 0000 предположительно статус mono для праворукой машины
	1110 0000 не известно
	*/
	oled.setCursor(18, 4);
	if (old[4] & 0x40) oled.print("Mono  T: "); // индикация mono режима
	else oled.print("Right T: ");
	uint8_t tr = old[4] & 0x1F;
	if (tr < 23) oled.print(temp_climat[tr]); // индикация температуры справа
	else { runfind("D4 темп"); return; }
	if (old[4] & 0xE0) { runfind("искл. D4"); return; }

	/*
	D5
	1000 0000 не известно всегда включен
	0100 0000 не известно
	0011 0000 циркуляция/обдув ветрового
	0000 1111 скорость вентилятора отопителя
	*/
	oled.setCursor(18, 5);
	oled.print("Fan:");
	uint8_t fan = old[5] & 0x0F;
	if (fan > 9) { runfind("D5 fan"); return; }
	if (!fan) oled.print("AUTO"); // отображает затененный вентилятор или отсутствие пропеллера(видимо не доработка инженеров на SMEG). Для индикации клавиши AUTO D5[X0] и D6[0X]
	else oled.print(fan - 1); // индикация скорости вентилятора
	uint8_t rejim = (old[5] & 0x30) >> 4;
	if (rejim == 0) {}
	else if (rejim == 1) oled.print(" F"); // см D0
	else if (rejim == 3) oled.print(" Circ");
	else { runfind("D5 режим"); return; }
	if (old[5] & 0x40) { runfind("искл. D5"); return; }
	if ((old[5] & 0x80) != 0x80) { runfind("искл. D5"); return; }

	/*
	D6
	1111 0000 состояние распределительных заслонок
	0000 1111 не известно
	*/
	oled.setCursor(18, 6);
	uint8_t zaslonki = old[6] >> 4;
	if (zaslonki == 0) oled.print("AUTO"); // режим автоматических заслонок
	else if (zaslonki == 1) oled.print("F");
	else if (zaslonki == 2) drawIcon7x7(151); // вниз
	else if (zaslonki == 3) drawIcon7x7(149); // вперед
	else if (zaslonki == 4) drawIcon7x7(152); // вверх
	else if (zaslonki == 5) { drawIcon7x7(149); oled.print(" "); drawIcon7x7(151); }
	else if (zaslonki == 6) { drawIcon7x7(152); oled.print(" "); drawIcon7x7(151); }
	else if (zaslonki == 7) { drawIcon7x7(152); oled.print(" "); drawIcon7x7(149); }
	else if (zaslonki == 8) { drawIcon7x7(152); oled.print(" "); drawIcon7x7(149); oled.print(" "); drawIcon7x7(151); }
	else { runfind("D6 заслонки"); return; }
	if ((old[6] & 0x0F) != 0x04) { runfind("искл. D6"); return; }

}

