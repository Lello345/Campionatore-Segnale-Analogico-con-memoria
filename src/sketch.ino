// Author = Leonardo Marcorè
// Universita' di camerino
// Corso: Architettura degli elaboratori
//MIT License

//Copyright (c) 2025 Lello345

//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#include <EEPROM.h>  // Libreria per usare la EEPROM interna

// === CONFIGURAZIONE ===
#define ANALOG_PIN A0            // Pin analogico da cui leggere
#define MAX_SAMPLES 816         // Numero massimo di campioni (1KB EEPROM / 5 byte ogni 4 campioni)
#define MAX_FREQ 1000            // Frequenza massima di campionamento (Hz)

// === VARIABILI GLOBALI (usate anche negli interrupt) ===
volatile bool sampling = false;           // Flag che indica se il campionamento è attivo
volatile uint16_t sampleBuffer[4];        // Buffer temporaneo per 4 campioni da 10 bit
volatile uint8_t bufferIndex = 0;         // Indice nel buffer da 0 a 3
volatile uint16_t totalSamples = 0;       // Campioni da acquisire (impostato da utente)
volatile uint16_t samplesCaptured = 0;    // Campioni già acquisiti
volatile uint16_t eepromAddr = 0;         // Indirizzo corrente in EEPROM

// Copie "non volatile" per il loop
uint16_t targetSamples = 0;
uint16_t sampleFreq = 0;

// === CONFIGURAZIONE TIMER1 PER FREQUENZA DI CAMPIONAMENTO ===
void setupTimer(uint16_t freq) {
  noInterrupts();        // Disabilita gli interrupt durante la configurazione
  TCCR1A = 0;            // Timer/Counter Control Register A = 0 (modalità normale)
  TCCR1B = 0;            // Timer/Counter Control Register B = 0
  TCNT1 = 0;             // Azzeriamo il contatore

  // Calcola valore di confronto per la frequenza desiderata
  // 16000000UL (UL =unsigned long, per garantire che il calcolo sia fatto a 32 bit) 
  // È il valore della frequenza del clock della CPU su Arduino UNO: 16 MHz
  // /8 È la divisione per il prescaler del Timer1 
  // Il prescaler "rallenta" il clock: se usi 8, ogni tick del timer dura 0.5 µs
  // freq è la frequenza di campionamento desiderata (in Hz)
  // Un interrupt ogni 1/freq secondi, quindi calcoli quanti tick servono per quel periodo
  uint32_t compareMatch = (16000000UL / 8) / freq - 1;
  // OCR1A è il registro di confronto del Timer1
  OCR1A = compareMatch;           // Valore di confronto per raggiungere il tempo voluto

  TCCR1B |= (1 << WGM12);         // Modalità CTC (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11);          // Prescaler 8
  TIMSK1 |= (1 << OCIE1A);        // Abilita interrupt sul confronto
  interrupts();                   // Riabilita gli interrupt
}

// Ferma Timer1
void stopTimer() {
  TIMSK1 &= ~(1 << OCIE1A);       // Disabilita l'interrupt di Timer1
}

// === CONFIGURA ADC (convertitore analogico-digitale) ===
void setupADC() {
  ADMUX = (1 << REFS0); // Riferimento AVcc, ingresso ADC0 (A0)
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2); 
  // Abilita ADC, interrupt ADC, prescaler 16
}

// Avvia una conversione ADC (manuale)
void startADCConversion() {
  ADCSRA |= (1 << ADSC); // Avvia conversione analogico-digitale
}

// === INTERRUPT: TIMER1 (trigger ogni N millisecondi) ===
ISR(TIMER1_COMPA_vect) {
  if (sampling && samplesCaptured < targetSamples) {
    startADCConversion(); // Avvia lettura analogica
  }
}

// === INTERRUPT: ADC CONVERSIONE COMPLETATA ===
ISR(ADC_vect) {
  uint16_t value = ADC; // Leggi valore a 10 bit (0-1023)

  // Salva nel buffer temporaneo
  sampleBuffer[bufferIndex++] = value;
  samplesCaptured++;

  // Se il buffer è pieno (4 campioni), salva 5 byte in EEPROM
  if (bufferIndex == 4) {
    writeSamplesToEEPROM();
    bufferIndex = 0;
  }

  // Se abbiamo finito tutti i campioni, stop
  if (samplesCaptured >= targetSamples) {
    sampling = false;
    Serial.println("Campionamento effettuato con successo.");
    stopTimer();
  }
}

// === SCRITTURA SU EEPROM: 4 campioni da 10 bit -> 5 byte ===
void writeSamplesToEEPROM() {
  if (eepromAddr + 5 > EEPROM.length()) return; // Evita scrittura fuori limite

  // Packing dei 4 valori da 10 bit (40 bit) in 5 byte
  uint8_t b0 = sampleBuffer[0] >> 2;
  uint8_t b1 = ((sampleBuffer[0] & 0x03) << 6) | (sampleBuffer[1] >> 4);
  uint8_t b2 = ((sampleBuffer[1] & 0x0F) << 4) | (sampleBuffer[2] >> 6);
  uint8_t b3 = ((sampleBuffer[2] & 0x3F) << 2) | (sampleBuffer[3] >> 8);
  uint8_t b4 = sampleBuffer[3] & 0xFF;

  // Scrittura sequenziale EEPROM
  EEPROM.update(eepromAddr++, b0);
  EEPROM.update(eepromAddr++, b1);
  EEPROM.update(eepromAddr++, b2);
  EEPROM.update(eepromAddr++, b3);
  EEPROM.update(eepromAddr++, b4);
}

// === SETUP INIZIALE ===
void setup() {
  Serial.begin(9600);
  setupADC();         // Inizializza ADC
  sei();              // Abilita global interrupt
  Serial.println("Pronto. Comandi: start <freq> <num>, stop, status, dump");
}

// === LOOP: GESTIONE COMANDI SERIALI ===
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');  // Legge riga da seriale
    cmd.trim();

    // === Comando: START <frequenza> <campioni> ===
    if (cmd.startsWith("start")) {
      if (sampling) {
        Serial.println("Errore: già in acquisizione.");
        return;
      }

      // Parsing argomenti
      int space1 = cmd.indexOf(' ');
      int space2 = cmd.indexOf(' ', space1 + 1);
      if (space1 == -1 || space2 == -1) {
        Serial.println("Uso: start <frequenza_Hz> <num_campioni>");
        return;
      }
      
      sampleFreq = cmd.substring(space1 + 1, space2).toInt();
      targetSamples = cmd.substring(space2 + 1).toInt();

      // Validazione parametri
      if (sampleFreq < 1 || sampleFreq > MAX_FREQ) {
        Serial.println("Frequenza non valida (1-1000 Hz)");
        return;
      }

      if (targetSamples > MAX_SAMPLES) {
        Serial.println("Troppi campioni richiesti (max 816)");
        return;
      }

      // Reset stato
      eepromAddr = 0;
      bufferIndex = 0;
      samplesCaptured = 0;
      sampling = true;
      setupTimer(sampleFreq);     // Avvia timer con la frequenza
      Serial.println("Campionamento avviato.");
    }

    // === Comando: STOP ===
    else if (cmd == "stop") {
      sampling = false;
      stopTimer();
      Serial.println("Campionamento interrotto.");
    }

    // === Comando: STATUS ===
    else if (cmd == "status") {
      if (sampling) {
        Serial.print("Campionamento in corso: ");
        Serial.print(samplesCaptured);
        Serial.print("/");
        Serial.println(targetSamples);
      } else {
        Serial.println("Campionamento fermo.");
      }
    }

    // === Comando: DUMP ===
    else if (cmd == "dump") {
      Serial.println("Contenuto EEPROM (campioni):");
      for (int i = 0; i < samplesCaptured; i += 4) {
        // Lettura 5 byte da EEPROM
        uint8_t b0 = EEPROM.read(i * 5 / 4 + 0);
        uint8_t b1 = EEPROM.read(i * 5 / 4 + 1);
        uint8_t b2 = EEPROM.read(i * 5 / 4 + 2);
        uint8_t b3 = EEPROM.read(i * 5 / 4 + 3);
        uint8_t b4 = EEPROM.read(i * 5 / 4 + 4);

        // Decodifica 5 byte -> 4 valori a 10 bit
        uint16_t s0 = ((b0 << 2) | (b1 >> 6)) & 0x3FF;
        uint16_t s1 = (((b1 & 0x3F) << 4) | (b2 >> 4)) & 0x3FF;
        uint16_t s2 = (((b2 & 0x0F) << 6) | (b3 >> 2)) & 0x3FF;
        uint16_t s3 = (((b3 & 0x03) << 8) | b4) & 0x3FF;

        // Stampa su seriale
        Serial.print("[" + String(i) + "]: ");
        Serial.print(s0); Serial.print(", ");
        Serial.print(s1); Serial.print(", ");
        Serial.print(s2); Serial.print(", ");
        Serial.println(s3);
      }
    }

    // === Comando sconosciuto ===
    else {
      Serial.println("Comando sconosciuto.");
    }
  }
}



