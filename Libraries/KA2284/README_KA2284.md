# KA2284 — symbol KiCad

Własny symbol KiCad 10 dla układu KA2284 w obudowie SIP-9.

## Pinout

| Pin | Funkcja |
|---:|---|
| 1 | D1 / LED 1 |
| 2 | D2 / LED 2 |
| 3 | D3 / LED 3 |
| 4 | D4 / LED 4 |
| 5 | GND |
| 6 | D5 / LED 5 |
| 7 | OUT — wyjście detektora/envelope |
| 8 | INPUT — wejście audio |
| 9 | VCC |

Domyślny footprint symbolu:

`Package_SIP:SIP-9_22.3x3mm_P2.54mm`

## Dodanie w polskim KiCadzie

1. Otwórz `Preferencje -> Zarządzanie bibliotekami symboli...`.
2. Przejdź do zakładki bibliotek projektu.
3. Kliknij ikonę dodania istniejącej biblioteki.
4. Wskaż plik `Libraries/KA2284.kicad_sym`.
5. Wstaw symbol przez `A` i wyszukaj `KA2284`.

Pinout i funkcje zostały przygotowane na podstawie datasheetu KA2284/KA2285.
