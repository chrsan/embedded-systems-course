# Uppgift 1

## Val av lösning

Har labbat med att köra både polling och ISR.

Detta projekt använder sig av polling men jag kommer att forsätta att labba
med ISR. Problemet jag hade med ISR var att när jag ska sätta tillbaka en
pinne (rad) till hög så körs interruptet igen. Har en idé jag ska testa med
att sätta typ till neg edge, men därav så lämnar jag in ett poll-baserad
lösningsförslag.

## Pull up

Alla pinnar är konfigurerade med att använda ESP32:s inbyggda pull up.

Det hade fungerat med att endast sätta det på kolumnpinnarna, men eftersom
pull up är enabled by default för alla pinnar (se `gpio_reset`) så känns det
bra att ha det enabled för samtliga pinnar.
