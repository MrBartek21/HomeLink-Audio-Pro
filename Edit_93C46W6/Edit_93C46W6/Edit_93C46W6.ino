// Piny sterujące pamięcią 93C46
const int CS_PIN  = 10;
const int DI_PIN  = 11; // Do pinu 3 (DI) kości
const int DO_PIN  = 12; // Do pinu 4 (DO) kości
const int SK_PIN  = 13;

// Instrukcje dla trybu 16-bitowego (64 słowa x 16 bitów)
// Instrukcja składa się z bitu startu (1) + 2 bitów kodu operacji
const uint8_t OP_READ  = 0b110; // Start(1) + Op(10)
const uint8_t OP_WRITE = 0b101; // Start(1) + Op(01)
const uint8_t OP_ERASE = 0b111; // Start(1) + Op(11)
const uint8_t OP_EWEN  = 0b100; // Start(1) + Op(00) -> podinstrukcja EWEN to 0011XXXX

void setup() {
  Serial.begin(115200);
  
  pinMode(CS_PIN, OUTPUT);
  pinMode(SK_PIN, OUTPUT);
  pinMode(DI_PIN, OUTPUT);
  pinMode(DO_PIN, INPUT);
  
  digitalWrite(CS_PIN, LOW);
  digitalWrite(SK_PIN, LOW);
  digitalWrite(DI_PIN, LOW);
  
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    // Odrzuć znaki nowej linii
    if (cmd == '\n' || cmd == '\r') return;
    
    switch (cmd) {
      case '1':
        readAll();
        break;
      case '2':
        eraseAll();
        break;
      case '3':
        writeWsad();
        break;
      default:
        printMenu();
        break;
    }
  }
}

void printMenu() {
  Serial.println(F("\n--- PROGRAMATOR EEPROM 93C46 (16-bit) ---"));
  Serial.println(F("Wpisz cyfre i nacisnij Enter:"));
  Serial.println(F("1 - Odczytaj cala zawartosc"));
  Serial.println(F("2 - Wyczysc pamiec (Format / Erase All)"));
  Serial.println(F("3 - Wgraj wsad testowy dla CM6206-LX"));
}

// Funkcja wysyłająca określoną liczbę bitów do pamięci
void shiftOutBits(uint32_t data, int bitCount) {
  for (int i = bitCount - 1; i >= 0; i--) {
    digitalWrite(DI_PIN, (data >> i) & 1);
    digitalWrite(SK_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SK_PIN, LOW);
    delayMicroseconds(5);
  }
}

// Odblokowanie zapisu/kasowania (EWEN)
void enableWrite() {
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(5);
  
  shiftOutBits(OP_EWEN, 3);          // Start + Opcode
  shiftOutBits(0b110000, 6);         // Adres dla EWEN (11XXXX dla 64 słów)
  
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
}

// Odczyt jednego słowa 16-bitowego z podanego adresu (0-63)
uint16_t readWord(uint8_t address) {
  uint16_t data = 0;
  
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(5);
  
  shiftOutBits(OP_READ, 3);          // Start + Opcode
  shiftOutBits(address & 0x3F, 6);   // 6 bitów adresu
  
  // Czytanie 16 bitów danych
  for (int i = 0; i < 16; i++) {
    digitalWrite(SK_PIN, HIGH);
    delayMicroseconds(5);
    data = (data << 1) | digitalRead(DO_PIN);
    digitalWrite(SK_PIN, LOW);
    delayMicroseconds(5);
  }
  
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  return data;
}

// Zapis jednego słowa 16-bitowego pod wskazany adres
void writeWord(uint8_t address, uint16_t value) {
  enableWrite();
  
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(5);
  
  shiftOutBits(OP_WRITE, 3);         // Start + Opcode
  shiftOutBits(address & 0x3F, 6);   // Adres
  shiftOutBits(value, 16);           // Dane
  
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  
  // Oczekiwanie na zakończenie wewnętrznego cyklu zapisu (Ready/Busy)
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(5);
  unsigned long startTime = millis();
  while (digitalRead(DO_PIN) == LOW) {
    if (millis() - startTime > 100) break; // Timeout ochronny
  }
  digitalWrite(CS_PIN, LOW);
}

// ODCZYT CAŁEJ PAMIĘCI
void readAll() {
  Serial.println(F("\n--- ZAWARTOSC EEPROM (Adres: Wartosc HEX) ---"));
  for (uint8_t addr = 0; addr < 64; addr++) {
    uint16_t val = readWord(addr);
    
    // Formatowanie wyświetlania do ładnej tabeli hex
    if (addr < 10) Serial.print("0");
    Serial.print(addr);
    Serial.print(": 0x");
    if (val < 0x1000) Serial.print("0");
    if (val < 0x0100) Serial.print("0");
    if (val < 0x0010) Serial.print("0");
    Serial.println(val, HEX);
  }
  printMenu();
}

// KASOWANIE CAŁEJ PAMIĘCI
void eraseAll() {
  Serial.print(F("\nCzyszczenie pamieci... "));
  enableWrite();
  
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(5);
  
  shiftOutBits(0b100, 3);            // Start + Opcode dla podinstrukcji (00)
  shiftOutBits(0b100000, 6);         // Kod dla ERAL (Erase All): 10XXXX
  
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  
  // Czekaj na koniec operacji kasowania
  digitalWrite(CS_PIN, HIGH);
  while (digitalRead(DO_PIN) == LOW);
  digitalWrite(CS_PIN, LOW);
  
  Serial.println(F("ZAKONCZONE. Wszystkie komorki maja teraz wartosc 0xFFFF."));
  printMenu();
}

// POPRAWIONY ZAPIS DLA CM6206-LX (Kolejność bajtów Little-Endian)
void writeWsad() {
  Serial.println(F("\nZapisywanie OFICJALNEGO wsadu [HomeLink Audio Pro]..."));
  
  // KROK 1: Sygnatura i identyfikatory (Porządek sprawdzony sprzętowo)
  writeWord(0x00, 0x434D); // Sygnatura "CD"
  writeWord(0x01, 0x0D8C); // Vendor ID: C-Media
  writeWord(0x02, 0x0102); // Product ID: CM6206
  
  // KROK 2: Tekst "HomeLink Audio Pro" w formacie Little-Endian
  // W każdym słowie: prawy bajt to pierwsza litera, lewy bajt to druga litera.
  
  // "HomeLink "
  writeWord(0x03, 0x6F48); // 'o' (0x6F) + 'H' (0x48) -> "Ho"
  writeWord(0x04, 0x656D); // 'e' (0x65) + 'm' (0x6D) -> "me"
  writeWord(0x05, 0x694C); // 'i' (0x69) + 'L' (0x4C) -> "Li"
  writeWord(0x06, 0x6B6E); // 'k' (0x6B) + 'n' (0x6E) -> "nk"
  writeWord(0x07, 0x4120); // 'A' (0x41) + Spacja (0x20) -> " A"
  
  // "Audio Pro"
  writeWord(0x08, 0x6475); // 'd' (0x64) + 'u' (0x75) -> "ud"
  writeWord(0x09, 0x6F69); // 'o' (0x6F) + 'i' (0x69) -> "io"
  writeWord(0x0A, 0x5020); // 'P' (0x50) + Spacja (0x20) -> " P"
  writeWord(0x0B, 0x6F72); // 'o' (0x6F) + 'r' (0x72) -> "ro"

  // KROK 3: Wyczyszczenie wszystkich pozostałych komórek pamięci do samego końca
  for (uint8_t i = 0x0C; i < 64; i++) {
    writeWord(i, 0x0000);
  }

  Serial.println(F("ZAKONCZONE! Sprawdź odczyt (opcja 1) przed wlutowaniem na stałe."));
  printMenu();
}

