# SK04 - Maze Solver

## Jak działa rozwiązanie
Robot porusza sie dyskretnie (kratka po kratce o 12 cm). Po napotkaniu ściany przed sobą zatrzymuje się i wykonuje skręt o 90 stopni.

**Korekcja jazdy (`DriveCorrector`):** W czasie jazdy na wprost w zależności od odczytów z sensorów korygowana jest prędkość kół na podstawie ścian bocznych lub różnicy enkoderów kół, gdy nie ma ścian po bokach.

**Korekcja błędów odometrii:** Ponieważ błędy enkoderów sumują się na zakrętach, robot wykorzystuje dodatkowe mechanizmy korekcji toru jazdy: prostopadłe wyrównanie do ściany przedniej przed wykonaniem skrętu (`align_to_front_wall`), detekcję końca ściany bocznej do odmierzenia środka skrzyżowania (`EDGE_OFFSET`) oraz kalibrację kąta obrotu o dobrany mnożnik.

## Strategie poruszania się po labiryncie
1. **Strategia Prawej/Lewej Ręki**
* Algorytm rozwiązania labiryntów z priorytetem wyboru drogi w prawą/ lewą stronę. Strategią `right_hand_strategy` robot przechodzi labirynt 1 w 42.50 s, a labirynt 2 w 100.38 s. Strategią `left_hand_strategy` natomiast w 15.66 i 116.54 s.

2. **Dynamic Flood Fill**
* Robot nie ma wpisanych z góry wymiarów labiryntu. Startuje z hipotezą mapy `2x2` i zakłada, że meta znajduje się w przeciwległym rogu `(size-1, size-1)`. Gdy widzi otwarte przejście poza dotychczasowe wymiary lub dochodzi do zakładanego rogu bez wykrycia mety, powiększa rozmiar mapy i przelicza komórki metodą BFS od pola celu. Robot pokonuje labirynt 1 w 15.66 s, a labirynt 2 w 49.44 s.

2. **Flood Fill**
* Dodatkowo zaimplementowano wersję ze znanymi wymiarami i współrzędnymi mety. Algorytm działa również dla labiryntów, gdzie meta nie leży w przeciwległym rogu.

## Co sprawiło największą trudność
* **Zaprojektowanie stabilnej korekcji toru i dobranie odpowiednich współczynników:** Ze względu na bardzo wąski korytarz oraz błędy odczytów z sensorów i enkoderów zaprojektowanie stabilnego regulatora w DriveCorrector wymagało obsłużenia czterech osobnych trybów pracy (dwie ściany, lewa, prawa, brak ścian - jazda na enkoderach) i dobrania wzmocnień (KP_BOTH, KP_ONE, KP_ENC). Zbyt duże wartości powodowały oscylacje i zahaczanie o ściany przy maksymalnej prędkości, a zbyt małe nie korygowały wystarczająco błędów i również powodowały zahaczanie o ściany.

* **Utrzymanie toru jazdy przy znikających ścianach:** W momencie mijania krawędzi ściany bocznej (np. przy wjeździe na skrzyżowanie) spadek odczytu sensora był błędnie odczytywany jako gwałtowne oddalenie się od ściany, co powodowało uruchomienie się korekcji i skręt robota w stronę otwartego przejścia. Rozwiązałam to przez wykrywanie spadku liczby widocznych dla robota ścian (flaga `walls_lost`) oraz wykorzystanie krawędzi ściany jako punktu do odmierzania stałego dystansu (`EDGE_OFFSET = 0.05`) do środka pola.

## Co poprawiłabym
* **Płynniejsza korekcja przy dojeżdżaniu do ścian:** Obecnie wyrównanie prostopadłe (`align_to_front_wall`) odbywa się dopiero po całkowitym zatrzymaniu robota przed ścianą, co wydłuża czas przejazdu. Można by połączyć korektę kąta i dystansu z fazą hamowania w `step_one_cell_forward()`, dzięki czemu robot wytracałby prędkość i jednocześnie ustawiał się prostopadle jednym płynnym ruchem.

* **Skręt po łuku zamiast obrotu w miejscu:** Aktualnie robot musi zatrzymać się przed każdym skrętem. Docelowo robot mógłby wykonywać ten zakręt płynną jazdą po łuku poprzez zadawanie różnych prędkości obu kołom już od momentu wykrycia końca ściany bocznej. Pozwoliłoby to utrzymać prędkość robota i znacząco skrócić czas przejazdu.
