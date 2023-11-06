# MorseTrainer
An STM32 project to learn to send and interpret Morse Code by yourself or with a friend.
## Overview
### History
In the early 1830's, Samuel FB Morse and his colleagues developed an electrical telegraph system designed to transmit a natural language using only electrical pulses and the silence between.  It evolved to become one of the first long widely used near-instant communication methods, paving the way for radiography, digital communication, and ultimately the internet.
It's mostly obsolete today and typically only used by hobbyists, but I plan to take a journey back in time and see what it was like to communicate long distances back in the 19th century.

### The Code
Most everyone knows Morse code is transmitted using dots and dashes (or dits and dahs if you're really getting into character) but most don't have the code memorized or know the timing standards associated with it. Here is standardized International Morse Code table translating each character or number into its associated series of dots and dashes:

| Letter | Morse Code |
|--------|------------|
| A      | .-         |
| B      | -...       |
| C      | -.-.       |
| D      | -..        |
| E      | .          |
| F      | ..-.       |
| G      | --.        |
| H      | ....       |
| I      | ..         |
| J      | .---       |
| K      | -.-        |
| L      | .-..       |
| M      | --         |
| N      | -.         |
| O      | ---        |
| P      | .--.       |
| Q      | --.-       |
| R      | .-.        |
| S      | ...        |
| T      | -          |
| U      | ..-        |
| V      | ...-       |
| W      | .--        |
| X      | -..-       |
| Y      | -.--       |
| Z      | --..       |

| Number | Morse Code |
|--------|------------|
| 0      | -----      |
| 1      | .----      |
| 2      | ..---      |
| 3      | ...--      |
| 4      | ....-      |
| 5      | .....      |
| 6      | -....      |
| 7      | --...      |
| 8      | ---..      |
| 9      | ----.      |

Morse Code Timing Characteristics

- **Dot:** 1 time unit.
- **Dash:** 3 time units.
- **Space between symbols (dots and dashes) of the same letter:** 1 time unit.
- **Space between letters:** 3 time units.
- **Space between words:** 7 time units.


From the table, we can see that the timing is characterized by generic "time units" and doesn't have a specified length of time for a dot, dash, end of character, space, or end of word. First things first: click a button a bunch of times and see how many miliseconds is a comfortable dot or dash for me. 

## Signal Processing

## Buzzer

## RF Transceiver

## PCB

## Results

