# PLC-100
library repository for PLC-100 Esp32-C6

## Abilitazione Seriale su Platformio
board_build.usb_cdc = 1
board_build.hwids = [["0x303A", "0x1001"]]
monitor_port = /dev/ttyACM0   ; o COMx su Windows
monitor_speed = 115200

## Abilitazione su arduino
Tools → USB CDC On Boot → Enabled

# ESP32 PLC-100 – Test di collaudo

## Informazioni generali
- Operatore: ____________________
- Data: _________________________
- Board ID / S/N: _______________

## Riepilogo test

| ID | Funzione        | Descrizione breve                      | Esito (OK/FAIL) | Note                                        |
|----|-----------------|----------------------------------------|-----------------|---------------------------------------------|
| T1 | Analog IN       | Lettura AO1..AO4                       |       OK        | Test superato con codice Full_board_test    |
| T2 | Analog OUT      | PWM su AO1..AO4                        |       OK        | Test superato con codice Full_board_test    |
| T3 | Relè            | Attivazione sequenziale REL1..REL6     |       OK        | Mappatura corretta via PCA9557              |
| T4 | Digital IN      | Lettura D1..D4                         |       OK        | GPIO 4–5–10–11 funzionanti                  |
| T5 | Seriale         | Loopback Serial1                       |       OK        | Comunicazione stabile                       |
| T6 | WiFi            | Connessione a SSID e IP assegnato      |       OK        | DHCP assegnato correttamente                |
| T7 | Zigbee          | Avvio stack Zigbee / join rete         |       OK        |  Join rete confermato                       |

## Dettaglio misure Analog IN

| Canale | GPIO  | Valore ADC medio | Valore atteso | OK/FAIL | Note |
|--------|-------|------------------|---------------|---------|------|
| AO1    | 18    |                  |               |   OK    |      |
| AO2    | 19    |                  |               |   OK    |      |
| AO3    | 20    |                  |               |   OK    |      |
| AO4    | 21    |                  |               |   OK    |      |

## Dettaglio Relè

| Relè | Canale logico | Stato testato (ON/OFF) | OK/FAIL | Note |
|------|---------------|------------------------|---------|------|
| REL1 |               |        ON/OFF          |         |      |
| REL2 |               |        ON/OFF          |         |      |
| REL3 |               |        ON/OFF          |         |      |
| REL4 |               |        ON/OFF          |         |      |
| REL5 |               |        ON/OFF          |         |      |
| REL6 |               |        ON/OFF          |         |      |
