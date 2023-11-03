//*************************************************************************************************************
// Programa do Equipamento Unidade Terminal Remota (UTR) com Arduíno Mega usando Interface Serial             *
//                                                                                                            *
// Data: 22/10/2023                                                                                           *
//                                                                                                            *
// O programa foi modificado para operação normal usando a energia da rede e inversor off grid. Na falta da   *
// energia da rede, o sistema entra em modo off grid, ligando o Inversor 1, alimentando a carga 2 (Torre), e  *
// alimentando a Carga3 (Portão, Câmeras e Luzes da Entrada) se a tensão de 24Vcc for maior que 25V e se a    *
// Carga3 está habilitada para ligar na falta de CA. Também na falta de CA, dois paineis fotovoltaicos são    *
// conectados ao controlador de carga.                                                                        *
//                                                                                                            *
//*************************************************************************************************************

// Inclusao das bibliotecas usadas no programa
#include <EEPROM.h>
#include <TimerOne.h>
#include <TimeLib.h>
#include <SPI.h>             // needed for Arduino versions later than 0018
#include <EthernetUdp.h>     // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <Rtc_Pcf8563.h>
#include <Wire.h>


// Definicao das constantes de conversao dos valores analogicos
#define KMed00      5.3442   // Constante de Medida de Tensao CC: 0 a 30,11Vcc Barramento 24Vcc
#define Offset00    0.0
#define KMed01      1.0      // Reserva
#define Offset01    0.0
#define KMed02      9.137    // Constante da Medida de Temperatura do Driver do Inversor 1
#define Offset02    0.0
#define KMed03      5.5236   // Constante de Corrente CC: Circuitos de Corrente Continua
#define Offset03    0.0
#define KMed04     44.31     // Constante de Medida de Tensao CA: Saida Inversor 1
#define Offset04    0.0
#define KMed05     44.225    // Constante de Medida de Tensao CA: Rede
#define Offset05    0.0
#define KMed06     43.18     // Constante de Medida de Tensao CA: Saida Inversor 2
#define Offset06    0.0
#define KMed07      9.711    // Constante da Medida de Temperatura Transformador do Inversor 1
#define Offset07    0.0
#define KMed08      9.292    // Constante da Medida de Temperatura do Driver do Inversor 2
#define Offset08    0.0
#define KMed09      9.335    // Constante da Medida de Temperatura Transformador do Inversor 2
#define Offset09    0.0
#define KMed10      2.98     // Constante de Corrente CA: Saida do Inversor 2
#define Offset10    0.0
#define KMed11      3.6806   // Constante da Medida de Corrente da Saida da Fonte CC
#define Offset11    0.0
#define KMed12      2.2169   // Corrente CC: Entrada do Inversor 2 (2.5119)
#define Offset12    4.0
#define KMed13      19.41    // Constante de Corrente CA: Saida do Inversor 1
#define Offset13    0.0
#define KMed14      1.4585   // Constante de Corrente CA: Carga 3 (Rede ou Inversor 1)
#define Offset14    7.0
#define KMed15      2.305    // Constante de Corrente CC: Entrada do Inversor 1 (2.728)
#define Offset15    0.0

// Definicao dos numeros dos pinos para leitura das Entradas Analogicas (EA)
#define EA00         0   // EA00 - Tensao CC: Barramento Principal de 24 Volts
#define EA01         1   // EA01 - Reserva
#define EA02         2   // EA02 - Temperatura: Driver do Inversor 2 (1)
#define EA03         3   // EA03 - Corrente CC: Circuitos de Corrente Continua
#define EA04         4   // EA04 - Tensao CA: Saida Inversor (2) 1
#define EA05         5   // EA05 - Tensao CA: Rede
#define EA06         6   // EA06 - Tensao CA: Saida Inversor 1 (2)
#define EA07         7   // EA07 - Temperatura: do Transformador do Inversor 2 (1)
#define EA08         8   // EA08 - Temperatura: dos Drivers do Inversor 1 (2)
#define EA09         9   // EA09 - Temperatura: do Transformador do Inversor 1 (2)
#define EA10        10   // EA10 - Corrente CA: Saida do Inversor 1 (2)
#define EA11        11   // EA11 - Corrente CC: Saida da Fonte CC
#define EA12        12   // EA12 - Corrente CC: Entrada do Inversor 1 (2)
#define EA13        13   // EA13 - Corrente CA: Saida do Inversor 2 (1)
#define EA14        14   // EA14 - Corrente CA: Carga 3 (Rede ou Inversor 1)
#define EA15        15   // EA15 - Corrente CC: Entrada do Inversor 2 (1)

// Definicao dos numeros dos Pinos de Sinais Digitais (SD e ED)
#define BotaoSilencia    3  // I Botao para desligar a sirene de nivel baixo da caixa azul
#define DJEINV2          5  // I Disjuntor de Entrada 24Vcc do Inversor 2: Desligado = 0 / Ligado = 1
#define DJEINV1          4  // I Disjuntor de Entrada 24Vcc do Inversor 1: Desligado = 0 / Ligado = 1
#define EdCxAzCheia      6  // I Indic. da Caixa Azul Cheia (outro contato da boia de acionamento da bomba)
#define FonteCC2Lig      7  // I Indic. da Fonte CA => 24Vcc dos Circuitos CC: Lig. = 1 / Desl. = 0
#define SD18             8  // O Sirene de Aviso de Nivel Baixo da Caixa Azul: 1 = Liga / 0 = Desliga

//                      22     Reserva
#define FonteCC1Lig     23  // I Indic. da Fonte do Inversor 2 e Carregador de Bateria Lig. = 1 / Desl. = 0

#define SD06            24  // O SD06 - 
#define SD03            25  // O SD03 - 
#define SD07            26  // O SD07 - 
#define SD11            27  // O SD11 - 
#define SD08            28  // O SD08 - Controla o LED 1 (Verde)
#define SD04            29  // O SD04 - Controla o LED 2 (Amarelo): sinaliza falha de inversor
#define SD05            30  // O SD05 - Controla o LED 3 (Verde): sinaliza tensao 220VCA para as Cargas
#define SD09            31  // O SD09 - Controla o LED 4 (Verde)
#define SD12            32  // O SD12 - Controla o LED Azul 1
#define SD13            33  // O SD13 - Controla o LED Azul 2
#define SD14            34  // O SD14 - Controla o LED Azul 3
#define SD15            35  // O SD15 - Controla o LED Azul 4
#define SD00            38  // O SD00 - 0 => CT1 = Rede / 1 => CT1 = Inversor 1
#define SD01            39  // O SD01 - 0 => Desliga Inv2 / 1 => Liga Inv 2
#define SD02            40  // O SD02 - 0 => CT3 = Rede / 1 => CT3 = Inversor

#define ChLR            41  // I Chave Local = 0 / Remoto = 1 => (primeira chave de baixo para cima)
#define ChEN            42  // I Chave Economia = 0 / Normal = 1 => (segunda chave de baixo para cima)
#define ChOnOff         43  // I Chave OnGrid = 0 / OffGrid = 1 => (terceira chave de baixo para cima)
#define ChPrB           44  // I Chave de Habilitacao de Cargas Posicao Baixo = 1
#define ChPrC           45  // I Chave de Habilitacao de Cargas Posicao Cima = 1

#define SD10            46  // O SD10 - 0 => Desliga Inversor 2 / 1 => Liga Inversor 2
#define SD16            47  // O SD16 - 0 => Paineis no Inversor On Grid / 1 => Paineis nos Controladores de Carga
#define SD17            48  // O SD17 - 0 => CT2 = Rede / 1 => CT2 = Inversor 1

#define CircuitoBoia    49  // I Circuito de Alimentacao da Boia da Cx Azul: Desligado = 0 / Ligado = 1
#define BoiaCxAzul      50  // I Retorno da Boia da Cx Azul: Cheia = 0 / Precisa Encher = 1
#define BoiaCxAzNivBx    2  // I Indicador de Nivel baixo na Caixa Azul
#define CircuitoBomba   51  // I Circuito de Alimentacao da Bomba: Desligado = 0 / Ligado = 1
#define AlimRedeBomba   52  // I Alimentacao CA Rede para a Bomba: Desligado = 0 / Ligado = 1

#define TCMEA    1000   // Tempo em ms de Ciclo para Calculo da Media das Entradas Analogicas
#define NCMED     420   // Numero de segundos para calculo da media estendida de tensao do barramento
#define TECT1    2000   // Tempo em ms de Espera para Transferir na Chave de Transferencia 1 (CT1)
#define TECT2    1000   // Tempo em ms de Espera para Transferir na Chave de Transferencia 2 (CT2)
#define TECT3    5000   // Tempo em ms de Espera para Transferir na Chave de Transferencia 3 (CT3)
#define TECT4    5000   // Tempo em ms de Espera para Transferir na Chave de Transferencia 4 (CT4)
#define TECTIv1  5000   // Tempo em ms de Espera para Conectar o Inversor 1 nas Chaves de Transferencia
#define TECTIv2  5000   // Tempo em ms de Espera para Conectar o Inversor 2 nas Chaves de Transferencia
#define TETR      100   // Tempo em ms de Espera para Transmitir a Resposta pela Ethernet
#define TRINV   20000   // Tempo em ms de Espera para o Inversor voltar de subtensao
#define TRRede     60   // Tempo em segundos de Espera de Estabilizacao de Tensao da Rede apos o Retorno
#define TFRede      5   // Tempo em segundos para confirmar Falta de CA

#define VMinBat     2300  // Tensao Média Minima da Bateria para Desligar o Inversor 1 na Falta de CA
#define VMinBatLig  2400  // Tensão Média Minima da Bateria para Ligar o Inversor 1 na Falta de CA

#define VTRNFCA 15000   // Valor da Tensao de Transicao do Estado Normal para Falta CA da Rede (V x 100)
#define VTRFNCA 18000   // Valor da Tensao de Transicao do Estado de Falta CA para Normal da Rede (V x 100)

#define VMinInv1 17000  // Valor da Tensao minima CA de saida do Inversor 1 (170,00 VCA 60Hz)
#define VMaxInv1 23000  // Valor da Tensao maxima CA de saida do Inversor 1 (230,00 VCA 60Hz)
#define VMinInv2 19000  // Valor da Tensao minima CA de saida do Inversor 2 (190,00 VCA 60Hz)
#define VMaxInv2 23000  // Valor da Tensao maxima CA de saida do Inversor 2 (230,00 VCA 60Hz)

#define IMaxInv1  2000  // Valor da corrente maxima de entrada do Inversor 1 (20,00 Amperes)
#define IMaxInv2  2000  // Valor da corrente maxima de entrada do Inversor 2 (20,00 Amperes)

#define LiTDInv  5500   // Limite de Temperatura do Driver dos Inversores (55,00 graus)
#define LiTTInv  5500   // Limite de Temperatura do transformador do Inversores (55,00 graus)

#define LIMICC3   100   // Limite Inferior para deteccao de Carga 3 Desligada
#define LIMSCC3   400   // Limite Superior para deteccao de Carga 3 Ligada
#define LIMFCC3  1800   // Limite para deteccao de Carga 3 em Falha

#define VLAut4   2700   // Tensao que habilita ligar a Carga 4 no Modo Automatico
#define VDAut4   2400   // Tensao que desabilita ligar a Carga 4 no Modo Automatico
#define VLAut3   2700   // Tensao que habilita ligar a Carga 3 no Modo Automatico
#define VDAut3   2400   // Tensao que desabilita ligar a Carga 3 no Modo Automatico
#define VLAut2   2700   // Tensao que habilita liga a Carga2 no Modo Automatico
#define VDAut2   2400   // Tensao que desabilita ligar a Carga2 no Modo Automatico
#define VLAut1   2700   // Tensao que habilita ligar a Carga 1 no Modo Automatico
#define VDAut1   2400   // Tensao que desabilita ligar a Carga 1 no Modo Automatico

#define TmpCPEMax  14400

#define BTPP       50       // Base de Tempo do Ciclo de Execucao do Programa Principal (ms)
#define BaseTempo  999673   // 1000000


//*************************************************************************************************************
//                                                                                                            *
//                                       Declaracao das Variaveis                                             *
//                                                                                                            *
//*************************************************************************************************************
//
// Constantes

const boolean OpOnGrid = true;   // Define modo de operação do programa com prioridade On Grid

const int NCCMEA = TCMEA/BTPP;   // Numero de Ciclos para Calculo da Media das Entradas Analogicas
const int NCTRCT1 = TECT1/BTPP;  // Numero de Ciclos de Espera para Transferir na CT1
const int NCTRCT2 = TECT2/BTPP;  // Numero de Ciclos de Espera para Transferir na CT2
const int NCTRCT3 = TECT3/BTPP;  // Numero de Ciclos de Espera para Transferir na CT3
const int NCTRCT4 = TECT4/BTPP;  // Numero de Ciclos de Espera para Transferir na CT4
const int NCTIv1 = TECTIv1/BTPP; // Numero de Ciclos de Espera para conectar os Inversor 1
const int NCTIv2 = TECTIv2/BTPP; // Numero de Ciclos de Espera para conectar os Inversor 2
const int NCET = TETR/BTPP;      // Numero de Ciclos de Espera para Transmitir a Resposta pela Ethernet
const int CTRI = TRINV/BTPP;     // Numero de Ciclos de Espera para o Inversor voltar de subtensao
const int NCDSLFT = 5000 / BTPP; // Numero de Ciclos de Espera para religar a fonte
const int NCCS = 1000 / BTPP;    // Numero de Ciclos entre as chamadas de comunicacao serial

// Dia de Janeiro          1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31
const int NasSolJan[] = { 544, 545, 545, 546, 547, 547, 548, 548, 549, 550, 550, 551, 551, 552, 553, 553, 554, 554, 555, 556, 556, 557, 557, 558, 558, 559, 559, 600, 600, 601, 602};
const int PorSolJan[] = {1845,1845,1846,1846,1846,1847,1847,1847,1847,1848,1848,1848,1848,1848,1848,1848,1848,1849,1849,1849,1849,1849,1849,1848,1848,1848,1848,1848,1848,1848,1847};

// Dia de Fevereiro        1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29
const int NasSolFev[] = { 602, 603, 603, 603, 604, 604, 605, 605, 606, 606, 607, 607, 607, 608, 608, 608, 609, 609, 609, 610, 610, 610, 611, 611, 611, 612, 612, 612, 612};
const int PorSolFev[] = {1847,1847,1847,1846,1846,1846,1846,1845,1845,1844,1844,1844,1843,1843,1842,1842,1841,1841,1840,1840,1839,1839,1838,1838,1837,1836,1836,1835,1835};

// Dia de Marco            1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31
const int NasSolMar[] = { 613, 613, 613, 613, 614, 614, 614, 614, 614, 615, 615, 615, 615, 615, 615, 616, 616, 616, 616, 616, 616, 617, 617, 617, 617, 617, 617, 617, 617, 618, 618};
const int PorSolMar[] = {1834,1833,1833,1832,1831,1831,1830,1829,1829,1828,1827,1826,1826,1825,1824,1824,1823,1822,1821,1821,1820,1819,1818,1818,1817,1816,1815,1815,1814,1813,1812};

// Dia de Abril            1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30
const int NasSolAbr[] = { 618, 618, 618, 618, 618, 619, 619, 619, 619, 619, 619, 620, 620, 620, 620, 620, 620, 621, 621, 621, 621, 621, 622, 622, 622, 622, 622, 623, 623, 623};
const int PorSolAbr[] = {1812,1811,1810,1810,1809,1808,1807,1807,1806,1805,1805,1804,1803,1803,1802,1801,1801,1800,1759,1759,1758,1758,1757,1757,1756,1756,1755,1754,1754,1753};

// Dia de Maio             1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31
const int NasSolMai[] = { 623, 624, 624, 624, 624, 625, 625, 625, 625, 626, 626, 626, 627, 627, 627, 628, 628, 628, 628, 629, 629, 629, 630, 630, 630, 631, 631, 631, 632, 632, 632};
const int PorSolMai[] = {1753,1753,1752,1752,1751,1751,1750,1750,1750,1749,1749,1749,1748,1748,1748,1748,1747,1747,1747,1747,1747,1746,1746,1746,1746,1746,1746,1746,1746,1746,1746};

// Dia de Junho            1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30
const int NasSolJun[] = { 633, 633, 633, 634, 634, 634, 635, 635, 635, 636, 636, 636, 636, 637, 637, 637, 637, 638, 638, 638, 638, 639, 639, 639, 639, 639, 639, 640, 640, 640};
const int PorSolJun[] = {1746,1746,1746,1746,1746,1746,1746,1746,1746,1746,1746,1746,1746,1747,1747,1747,1747,1747,1748,1748,1748,1748,1748,1749,1749,1749,1749,1750,1750,1750};

// Dia de Julho            1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31
const int NasSolJul[] = { 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 639, 639, 639, 639, 639, 639, 639, 638, 638, 637, 637, 637, 636, 636};
const int PorSolJul[] = {1751,1751,1751,1751,1752,1752,1752,1753,1753,1753,1754,1754,1754,1754,1755,1755,1755,1756,1756,1756,1757,1757,1757,1757,1758,1758,1758,1758,1759,1759,1759};

// Dia de Agosto           1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31
const int NasSolAgo[] = { 636, 635, 635, 635, 634, 634, 633, 633, 632, 632, 631, 631, 630, 630, 629, 628, 628, 627, 627, 626, 625, 625, 624, 623, 623, 622, 621, 621, 620, 619, 618 };
const int PorSolAgo[] = {1759,1800,1800,1800,1800,1801,1801,1801,1801,1801,1802,1802,1802,1802,1802,1803,1803,1803,1803,1803,1803,1803,1804,1804,1804,1804,1804,1804,1804,1804,1804};

// Dia de Setembro         1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30
const int NasSolSet[] = { 618, 617, 616, 615, 615, 614, 613, 612, 612, 611, 610, 609, 609, 608, 607, 606, 605, 605, 604, 603, 602, 601, 601, 600, 559, 558, 557, 557, 556, 555};
const int PorSolSet[] = {1805,1805,1805,1805,1805,1805,1805,1805,1805,1805,1805,1805,1806,1806,1806,1806,1806,1806,1806,1806,1806,1806,1806,1806,1807,1807,1807,1807,1807,1807};

// Dia de Outubro          1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31
const int NasSolOut[] = { 554, 554, 553, 552, 551, 551, 550, 549, 549, 548, 547, 546, 546, 545, 544, 544, 543, 543, 542, 541, 541, 540, 540, 539, 539, 538, 538, 537, 537, 536, 536};
const int PorSolOut[] = {1807,1807,1807,1808,1808,1808,1808,1808,1808,1809,1809,1809,1809,1809,1809,1810,1810,1810,1810,1811,1811,1811,1811,1812,1812,1812,1813,1813,1813,1814,1814};

// Dia de Novembro         1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30
const int NasSolNov[] = { 535, 535, 535, 534, 534, 534, 533, 533, 533, 532, 532, 532, 532, 532, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531};
const int PorSolNov[] = {1814,1815,1815,1816,1816,1816,1817,1817,1818,1818,1819,1819,1820,1820,1821,1821,1822,1822,1823,1823,1824,1824,1825,1825,1826,1827,1827,1828,1828,1829};

// Dia de Dezembro         1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31
const int NasSolDez[] = { 532, 532, 532, 532, 532, 533, 533, 533, 534, 534, 534, 535, 535, 535, 536, 536, 537, 537, 538, 538, 539, 539, 540, 540, 541, 541, 542, 542, 543, 543, 544};
const int PorSolDez[] = {1829,1830,1831,1831,1832,1832,1833,1834,1834,1835,1835,1836,1836,1837,1837,1838,1839,1839,1840,1840,1841,1841,1842,1842,1842,1843,1843,1844,1844,1845,1845};

// Hora do Nascer e Por do Sol no dia Atual
int HorNasSol;
int HorPorSol;

// Variaveis de Hora e Minuto
int HHLB  =  900;   // Hora que habilita ligar o inversor da bomba
int HDLB  = 1530;   // Hora que desabilita ligar o inversor da bomba
int HHC1  =  700;   // Hora que habilita ligar a Carga 1
int HDC1  = 2300;   // Hora que desabilita ligar a Carga 1
int HHC2  =  700;   // Hora que habilita ligar a Carga 2
int HDC2  = 1730;   // Hora que desabilita ligar a Carga 2
int HHC3  =  730;   // Hora que habilita ligar a Carga 3
int HDC3  = 1700;   // Hora que desabilita ligar a Carga 3
int HHC4  =  900;   // Hora que habilita ligar a Carga 4
int HDC4  = 1530;   // Hora que desabilita ligar a Carga 4
int HDCAB = 1700;   // Hora que desabilita CA para a Bomba em modo geracao solar

// Medidas
double EA[16];        // Array com os Valores da Media das Entradas Analogicas
double EAT[16];       // Array com os acumuladores de calculo de media das entradas analogicas
double EAME0;         // Variavel com a Media estendida da Entrada Analogica 0 = Tensao 24Vcc Geral
double EAMET0;        // Variavel com o acumulador para calculo da media estendida da Tensao 24Vcc

double VBat;          // EA00 - Tensao CC: Barramento Principal de 24 Volts
double TDInv2;        // EA02 - Temperatura: Driver do Inversor 2
double ICirCC;        // EA03 - Corrente CC: Circuitos de Corrente Continua
double VSInv1;        // EA04 - Tensao CA: Saida Inversor 1
double VRede;         // EA05 - Tensao CA: Rede
double VSInv2;        // EA06 - Tensao CA: Saida Inversor 2
double TTInv2;        // EA07 - Temperatura: Transformador do Inversor 2
double TDInv1;        // EA08 - Temperatura: Drivers do Inversor 1
double TTInv1;        // EA09 - Temperatura: Transformador do Inversor 1
double ISInv1;        // EA10 - Corrente CA: Saida do Inversor 1
double ISFtcc;        // EA11 - Corrente CC: Saida da Fonte CC
double IEInv1;        // EA12 - Corrente CC: Entrada do Inversor 1
double ISInv2;        // EA13 - Corrente CA: Saida do Inversor 2
double ICg3;          // EA14 - Corrente CA: Carga 3
double IEInv2;        // EA15 - Corrente CC: Entrada do Inversor 2

// Variaveis de Estado
byte EstadoInversor1; // Inversor 1 - 1 = Ligado / 0 = Desligado
byte EstadoInversor2; // Inversor 2 - 1 = Ligado / 0 = Desligado
byte EstadoRede;      // Estado da Tensao da Rede
byte Estado24Vcc;     // Estado da Tensao 24Vcc
byte EstadoCarga3;    // Estado da Carga 3: 0 = Desligada / 1 = Ligada
byte EstadoCxAz;      // Estado da Caixa Azul: 0=Indefinido / 1=Prec Ench Niv Baixo / 2=Prec Ench Niv Norm/ 3=Cheia
byte ModoComando;     // 0 = Controle Local / 1 = Controle Remoto
byte ModoControle;    // 0 = Controle Manual / 1 = Controle Automatico das Cargas 2, 3 e 4
byte ModoControle1;   // 0 = Controle Manual / 1 = Controle Automatico da Carga 1
byte ModoOperacao;    // 0 = Economia de Bateria / 1 = Normal
byte Carga1;          // 0 = Desabilita Carga 1 / 1 = Habilita Carga 1
byte Carga2;          // 0 = Desabilita Carga 2 / 1 = Habilita Carga 2
byte Carga3;          // 0 = Desabilita Carga 3 / 1 = Habilita Carga 3
byte Carga4;          // 0 = Desabilita Carga 4 / 1 = Habilita Carga 4
byte BombaLigada;     // 0 = Bomba Desligada / 1 = Bomba Ligada
byte HabCABmb;

boolean OpOffGrid;    // Indica tipo de operação

boolean LigaFonte;    // Indica que a fonte CC deve ser ligada

// Variaveis de Alarme
byte FalhaInversor1;
byte SubTensaoInv1;
byte SobreTensaoInv1;
byte SobreTempDrInv1;
byte SobreTempTrInv1;
byte SobreCorrenteInv1;
byte DisjAbertoIv1;
byte FalhaInversor2;
byte SubTensaoInv2;
byte SobreTensaoInv2;
byte SobreTempDrInv2;
byte SobreTempTrInv2;
byte SobreCorrenteInv2;
byte DisjAbertoIv2;
byte FalhaCarga3;
byte CxAzNivBaixo;

// Indicadores Auxiliares
boolean EvtFaltaCA;
boolean HabCom;
boolean CxAzPrecEncher;
boolean CxAzCheia;

// Contadores de Tempo
unsigned int CTEA;         // Contador de media das entradas analogicas
unsigned int CTIV1;        // Contador de temporizacao para ativacao do Inversor 1
unsigned int CTIV2;        // Contador de temporizacao para ativacao do Inversor 1
unsigned int CTME;         // Contador de media estendida para decisoes do gerenciador de cargas
unsigned int CTRCA;        // Contador de temporizacao de retorno de CA da rede
unsigned int CTFCA;        // Contador de temporizacao para deteccao de Falta de CA
unsigned int CTCT1;        // Contador de temporizacao de transferencia da Chave de Transferencia 1
unsigned int CTCT2;        // Contador de temporizacao de transferencia da Chave de Transferencia 2
unsigned int CTCT3;        // Contador de temporizacao de transferencia da Chave de Transferencia 3
unsigned int CTCT4;        // Contador de temporizacao de transferencia da Chave de Transferencia 4
unsigned int CTIv1;        // Contador de temporizacao de conexao do Inversor 1
unsigned int CTIv2;        // Contador de temporizacao de conexao do Inversor 2
unsigned int CTRE;         // Contador de temporizacao de resposta da Ethernet
unsigned int CTCS;         // Contador de temporizacao para Comunicacao Serial
unsigned int CTRF;         // Contador de temporizacao para Reset da Fonte CC
unsigned int CTRFR;        // Contador de temporizacao para verificacao da fonte CC apos o reset

byte FlgTmpCT1;            // Indicador de temporizacao para transferencia de carga da CT1
byte FlgTmpCT2;            // Indicador de temporizacao para transferencia de carga da CT2
byte FlgTmpCT3;            // Indicador de temporizacao para transferencia de carga da CT3
byte FlgTmpCT4;            // Indicador de temporizacao para transferencia de carga da CT4
byte FlgTmpIv1;            // Indicador de temporizacao para ligar o Inversor 1
byte FlgTmpIv2;            // Indicador de temporizacao para ligar o Inversor 2 
byte FlgTmpFT;             // Indicador de temporizacao para desligar e ligar a fonte
byte FlgTmpFTR;            // Indicador de temporizacao para verificacao da fonte CC apos o reset

boolean HabTmpBombaLig;    // Flag de Habilitacao do Temporizador de Bomba Ligada
boolean HabTmpCxAzNvBx;    // Flag de Habilitacao do Temporizador de Caixa em Nivel Baixo
boolean Silencia;          // Flag que indica que o botao de silencia foi apertado
double TmpBombaLig;        // Contador de Segundos da Caixa Precisa Encher = Bomba Ligada
double TmpCxAzNvBx;        // Contador de Segundos do tempo que a caixa azul precisa encher

// Relogio
byte Hora;
byte Minuto;
byte Segundo;
byte Dia;
byte Mes;
int Ano;

byte HoraRec;
byte MinutoRec;
byte SegundoRec;
byte DiaRec;
byte MesRec;
int AnoRec;

// Flag que indica Acerto de Relogio pelo Host
byte AcertaRelogio;

boolean VemDoReset;

unsigned int Contador = 0;
byte HabAquisicao = 0;
byte MsgTr = 0;

// Buffers de Mensagens da Interface Serial1 de Comunicacao com o Concentrador Arduino Mega
byte MsgRxS1[16];  // Buffer para Recebimento da Mensagem de Solicitacao de Dados e Comando
byte MsgTxS1[64];  // Buffer para Envio do Estado de Todos os Pontos Digitais e Analogicos
byte EDbyte[8];    // Buffer para as Entradas Digitais em Modo Compacto
  
// buffers para enviar e receber dados pela interafce ethernet
char RxBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,

// An EthernetUDP instance to let us send and receive packets over UDP
//EthernetUDP Udp;

tmElements_t tm;

//init the real time clock
Rtc_Pcf8563 rtc;

 
//***********************************************************************************************************************
//                                                                                                                      *
//                                            Procedimentos de Inicializacao                                            *
//                                                                                                                      *
//***********************************************************************************************************************
//
void setup() {

  // Definir a Funcao dos Pinos de Entrada Digital
  pinMode(ChLR, INPUT);           // (ChLR) Chave Local = 0 / Remoto = 1 => (primeira chave de baixo para cima)
  pinMode(ChEN, INPUT);           // (ChEN) Chave Economia = 0 / Normal = 1 => (segunda chave de baixo para cima)
  pinMode(ChOnOff, INPUT);        // (ChOnOff) Chave OnGrid = 0 / OffGrid = 1 => (terceira chave de baixo para cima)
  pinMode(ChPrB, INPUT);          // (ChPrB) Chave de Habilitacao de Cargas Posicao Baixo = 1
  pinMode(ChPrC, INPUT);          // (ChPrC) Chave de Habilitacao de Cargas Posicao Cima = 1
  pinMode(DJEINV1, INPUT);        // (DJEINV1) Disjuntor de Entrada 24Vcc do Inversor 1: Desligado = 0 / Ligado = 1
  pinMode(DJEINV2, INPUT);        // (DJEINV2) Disjuntor de Entrada 24Vcc do Inversor 2: Desligado = 0 / Ligado = 1
  pinMode(CircuitoBoia, INPUT);   // (CircuitoBoia) Circuito de Alimentacao da Boia da Cx Azul: Desligado = 0 / Ligado = 1
  pinMode(BoiaCxAzul, INPUT);     // (BoiaCxAzul) Retorno da Boia da Cx Azul: Cheia = 0 / Precisa Encher = 1
  pinMode(CircuitoBomba, INPUT);  // (CircuitoBomba) Circuito de Alimentacao da Bomba: Desligado = 0 / Ligado = 1
  pinMode(AlimRedeBomba, INPUT);  // (AlimRedeBomba) Alimentacao CA Rede para a Bomba: Desligado = 0 / Ligado = 1
  pinMode(BoiaCxAzNivBx, INPUT);  // (BoiaCzAzNivBx) Indicador de Nivel baixo na Caixa Azul
  pinMode(BotaoSilencia, INPUT);  // (BotaoSilencia) Botao para silenciar a sirene de nivel baixo
  pinMode(EdCxAzCheia, INPUT);    // (EdCxAzCheia) Outro contato da boia principal da Caixa Azul - Caixa Cheia = 1
  pinMode(FonteCC1Lig, INPUT);    // (FonteCC1Lig) Indicador da Fonte do Inversor 2 e Carregador de Batreria Ligada
  pinMode(FonteCC2Lig, INPUT);    // (FonteCC2Lig) Indicador da Fonte 24Vcc dos circuitos CC Ligada
  
  // Definir a Funcao dos Pinos de Saida Digital
  pinMode(SD00, OUTPUT);
  pinMode(SD01, OUTPUT);
  pinMode(SD02, OUTPUT);
  pinMode(SD03, OUTPUT);
  pinMode(SD04, OUTPUT);
  pinMode(SD05, OUTPUT);
  pinMode(SD06, OUTPUT);
  pinMode(SD07, OUTPUT);
  pinMode(SD08, OUTPUT);
  pinMode(SD09, OUTPUT);
  pinMode(SD10, OUTPUT);
  pinMode(SD11, OUTPUT);
  pinMode(SD12, OUTPUT);
  pinMode(SD13, OUTPUT);
  pinMode(SD14, OUTPUT);
  pinMode(SD15, OUTPUT);
  pinMode(SD16, OUTPUT);
  pinMode(SD17, OUTPUT);
  pinMode(SD18, OUTPUT);
             
  // Inicia os Contadores
  CTEA = 0;
  CTIV1 = 0;
  CTME = 0;
  CTRCA = 0;
  CTFCA = TFRede;
  CTEA = 0;
  CTCT1 = 0;
  CTCT2 = 0;
  CTCT3 = 0;
  CTIv1 = 0;
  CTIv2 = 0;
  CTRE = 0;
  CTCS = 0;
  CTRF = 0;
  CTRFR = 0;

  // Inicia os flags de temporizacao
  FlgTmpCT1 = 0;
  FlgTmpCT2 = 0;
  FlgTmpCT3 = 0;
  FlgTmpCT4 = 0;
  FlgTmpIv1 = 0;
  FlgTmpIv2 = 0;
  FlgTmpFT = 0;
  FlgTmpFTR = 0;
      
  // Inicia os acumuladores das EAs com zero
  for (int i = 0; i < 16; i++) {
    EAT[i] = 0;
  }
  
  // Zera as Variaveis de Estado
  EstadoRede = 0;
  EstadoInversor1 = 0;
  EstadoInversor2 = 0;
  EstadoCxAz = 0;          // Inicia o Estado da Caixa Azul = Indefinido
  HabCABmb = 0;            // Inicia com a o CA da Bomba Desabilitado
  
  // Zera os Alarmes
  FalhaInversor1 = 0;
  SubTensaoInv1 = 0;
  SobreTensaoInv1 = 0;
  SobreTempDrInv1 = 0;
  SobreTempTrInv1 = 0;
  SobreCorrenteInv1 = 0;
  DisjAbertoIv1 = 0;
  FalhaInversor2 = 0;
  SubTensaoInv2 = 0;
  SobreTensaoInv2 = 0;
  SobreTempDrInv2 = 0;
  SobreTempTrInv2 = 0;
  SobreCorrenteInv2 = 0;
  DisjAbertoIv2 = 0;
  FalhaCarga3 = 0;
  CxAzNivBaixo = 0;
  EvtFaltaCA = false;
  HabCom = true;
    
  ModoComando = 1;         // Inicia o Modo de Comando Remoto
  ModoControle = 1;        // Inicia o Modo de Controle com a Carga3 habilitada na falta de CA
  ModoControle1 = 0;       // Inicia o Modo de Controle Manual da Carga 1
  ModoOperacao = 1;        // Inicia o Modo de Operacao Normal
  Carga1 = 0;              // Inicia a Carga 1 desabilitada
  Carga2 = 0;              // Inicia a Carga 2 desabilitada
  Carga3 = 0;              // Inicia a Carga 3 desabilitada
  Carga4 = 0;              // Inicia a Carga 4 desabilitada
  BombaLigada = 0;         // Inicia o indicador de Bomba Ligada
  LigaFonte = false;
  HabTmpBombaLig = false;
  HabTmpCxAzNvBx = false;
  Silencia = false;
  TmpBombaLig = 0;
  TmpCxAzNvBx = 0;

  OpOffGrid = false;
  
  AcertaRelogio = 0;

  VemDoReset = true;
  
  // Coloca todas as saidas digitais no estado inicial
  digitalWrite(SD00, LOW);
  digitalWrite(SD01, LOW);
  digitalWrite(SD02, LOW);
  digitalWrite(SD03, LOW);
  digitalWrite(SD04, LOW);
  digitalWrite(SD05, LOW);
  digitalWrite(SD06, LOW);
  digitalWrite(SD07, LOW);
  digitalWrite(SD08, LOW);
  digitalWrite(SD09, LOW);
  digitalWrite(SD10, LOW);
  digitalWrite(SD11, LOW);
  digitalWrite(SD12, LOW);
  digitalWrite(SD13, LOW);
  digitalWrite(SD14, LOW);
  digitalWrite(SD15, LOW);
  digitalWrite(SD16, LOW);
  digitalWrite(SD17, LOW);
  digitalWrite(SD18, LOW);
    
  // Inicia as Variaveis do relogio recebido
  HoraRec = 0;
  MinutoRec = 0;
  SegundoRec = 0;
  DiaRec = 1;
  MesRec = 1;
  AnoRec = 2017;
  
  // Inicia a Interfaces Seriais Assincronas
  Serial.begin(9600);   // Comunicacao com o IDE (Carga de Programa e Monitor Serial)
  Serial1.begin(9600);  // Comunicacao com o Concentrador Arduino Mega
      
  // Inicia o acumulador para calculo de media estendida
  EAMET0 = 0;
 
  // Espera a aquisicao analogica normalizar (5 ciclos de aquisicao analogica de 10 aquisicoes cada)
  for (int i = 0; i < 100; i++) {
    CarregaMedidas();
    delay(10);
  }

  Serial.print("Tensão Rede: ");
  Serial.println(VRede);
  
  // Inicia a variavel EstadoRede
  if (EA[5] > VTRNFCA) {
    EstadoRede = 1;
  }
      
  EAME0 = EA[0];     // Inicia o valor da media estendida da Tensao 24Vcc
  VerificaCarga3();  // Inicia o Estado da Carga 3
  VerifEstCxAz();    // Inicia os Sinalizadores de Estado da Caixa Azul

  // Inicia o timer e a chamada da rotina de interrupcao
  Timer1.initialize(BaseTempo); // A rotina Temporizacao eh chamada a cada BaseTempo microssegundos
  Timer1.attachInterrupt(Temporizacao);
 
} // Fim dos Procedimentos de Inicializacao


//***********************************************************************************************************************
//                                                                                                                      *
//                                          Inicio do Programa Principal                                                *
//                                                                                                                      *
//***********************************************************************************************************************
//
void loop() {
  
  // Calcula os Horarios de Habilitacao e Desabilitacao das Cargas CA
  CalculaHorarios();

  // Verifica o Estado das Chaves de Selecao de Modo de Operacao e Prioridade
  VerificaChaves();
  
  // Carrega as Medidas a partir das Entradas Analogicas (fisicas ou simuladas)
  CarregaMedidas();

  // Verifica o Estado da Tensao da Rede
  VerificaTensaoRede();
  
  // Verifica o Estado da Carga 3
  VerificaCarga3();
   
  // Verifica o Modo de Operacao e a Habilitacao das Cargas
  VerificaModoOp();
  
  // Verifica o funcionamento do Inversor 1
  //VerificaInversor1();

  // Verifica o funcionamento do Inversor 2
  VerificaInversor2();

  // Atualiza os estados da Caixa Azul
  VerifEstCxAz();

  // Verifica o funcionamento da Bomba d'Agua do Poco
  VerifCxAzBomba();

  // Controla o LEDs de Sinalizacao
  ControleLEDs();

  // Efetua os procedimentos de comunicacao serial com o Concentrador Arduino Mega
  ComunicacaoSerial();

  // Base de tempo entre ciclos de programa
  delay(BTPP);
    
} // Final do Loop Principal do Programa


//*********************************************************************************************************************
// Nome da Rotina: CalculaHorarios                                                                                    *
//                                                                                                                    *
// Funcao: calcula os horarios de habilitacao de desabilitacao das cargas CA                                          *
//                                                                                                                    *
// Entrada: nenhum                                                                                                    *
// Saida: nenhum                                                                                                      *
//*********************************************************************************************************************
//
void CalculaHorarios() {

  int mes = rtc.getMonth();
  int dia = rtc.getDay();

  switch (mes) {
    
    case 1:
      HorNasSol = NasSolJan[dia - 1];
      HorPorSol = PorSolJan[dia - 1];
    break;

    case 2:
      HorNasSol = NasSolFev[dia - 1];
      HorPorSol = PorSolFev[dia - 1];
    break;

    case 3:
      HorNasSol = NasSolMar[dia - 1];
      HorPorSol = PorSolMar[dia - 1];
    break;

    case 4:
      HorNasSol = NasSolAbr[dia - 1];
      HorPorSol = PorSolAbr[dia - 1];
    break;

    case 5:
      HorNasSol = NasSolMai[dia - 1];
      HorPorSol = PorSolMai[dia - 1];
    break;

    case 6:
      HorNasSol = NasSolJun[dia - 1];
      HorPorSol = PorSolJun[dia - 1];
    break;

    case 7:
      HorNasSol = NasSolJul[dia - 1];
      HorPorSol = PorSolJul[dia - 1];
    break;

    case 8:
      HorNasSol = NasSolAgo[dia - 1];
      HorPorSol = PorSolAgo[dia - 1];
    break;

    case 9:
      HorNasSol = NasSolSet[dia - 1];
      HorPorSol = PorSolSet[dia - 1];
    break;

    case 10:
      HorNasSol = NasSolOut[dia - 1];
      HorPorSol = PorSolOut[dia - 1];
    break;

    case 11:
      HorNasSol = NasSolNov[dia - 1];
      HorPorSol = PorSolNov[dia - 1];
    break;

    case 12:
      HorNasSol = NasSolDez[dia - 1];
      HorPorSol = PorSolDez[dia - 1];
    break;
    
  }

  int Soma;
  int Hora;
  int Minuto;
  
  HHC1 = HorNasSol;         // Hora que habilita ligar a Carga 1
  HDC1 = HorPorSol;         // Hora que desabilita ligar a Carga 1
    
  HHC2 = HorNasSol + 100;   // Hora que habilita ligar a Carga 2
  HDC2 = HorPorSol - 100;   // Hora que desabilita ligar a Carga 2
  
  HHC3 = HorNasSol + 100;   // Hora que habilita ligar a Carga 3
  HDC3 = HorPorSol - 200;   // Hora que desabilita ligar a Carga 3
  
  HHC4 = HorNasSol + 300;   // Hora que habilita ligar a Carga 4
  HDC4 = HorPorSol - 300;   // Hora que desabilita ligar a Carga 4
  
  HHLB = HorNasSol + 300;   // Hora que habilita ligar o inversor da bomba
  HDLB = HorPorSol - 300;   // Hora que desabilita ligar o inversor da bomba
  
  HDCAB = HorPorSol - 100;  // Hora que desabilita CA para a Bomba em modo geracao solar
  
}


//*********************************************************************************************************************
// Nome da Rotina: ComunicacaoSerial                                                                                  *
//                                                                                                                    *
// Funcao: verifica se tem requisicao pela interface serial e responde a solicitacao                                  *
//                                                                                                                    *
// Entrada: nenhum                                                                                                    *
// Saida: nenhum                                                                                                      *
//*********************************************************************************************************************
//
void ComunicacaoSerial() {

  byte NBMsgR = 14;  // Numero de Bytes da Mensagem de Recebida do Concentrador Arduino Mega
  byte NBMsgT = 60;  // Numero de Bytes da Mensagem de Resposta para o Concentrador Arduino Mega
  byte TmoIED;
  byte Comando;
  byte ch;
  
  int cont = 0;
  unsigned int CRC = 0xffff;
  byte lsb;

  // Se chegou uma mensagem com mais de 13 bytes, indicando uma mensagem de requisicao enviada pelo
  // Concentrador Arduino Mega, le os bytes recebidos pela Interface Serial1
  if (Serial1.available() > 13) {
    MsgRxS1[0] = Serial1.read();  // Endereco da UTR
    MsgRxS1[1] = Serial1.read();  // Funcao
    MsgRxS1[2] = Serial1.read();  // Codigo do Comando
    MsgRxS1[3] = Serial1.read();  // Reserva
    MsgRxS1[4] = Serial1.read();  // Hora
    MsgRxS1[5] = Serial1.read();  // Minuto
    MsgRxS1[6] = Serial1.read();  // Segundo
    MsgRxS1[7] = Serial1.read();  // Dia
    MsgRxS1[8] = Serial1.read();  // Mes
    MsgRxS1[9] = Serial1.read();  // Ano
    MsgRxS1[10] = Serial1.read(); // Reserva
    MsgRxS1[11] = Serial1.read(); // Reserva
    MsgRxS1[12] = Serial1.read(); // CRC Low
    MsgRxS1[13] = Serial1.read(); // CRC High

    // Limpa o buffer serial de bytes adicionais
    while (Serial1.available() > 0) {
      ch = Serial1.read();
    }
   
    //Serial.print("Recebida Msg Req. - ");

    // Calcula o CRC da mensagem recebida
    for (int j = 0; j < (NBMsgR - 2); j++) {
      CRC = CRC ^ MsgRxS1[j];
      for (int i = 0; i < 8; i++) {
        lsb = CRC & 0x0001;
        if (lsb == 0){
          CRC = CRC / 2;
        }
        if (lsb == 1) {
          CRC = CRC / 2;
          CRC = CRC^0xA001;
        }
      }
    }

    // Verifica se a mensagem recebida do Concentrador Arduino Mega chegou corretamente
    boolean CRC_OK = false;
    if ((MsgRxS1[NBMsgR - 2] == ByteL(CRC)) && (MsgRxS1[NBMsgR - 1] == ByteH(CRC))) {
        CRC_OK = true;
    }

    // Verifica os dois primeiros bytes da Mensagem (Endereco da UTR = 1 e Funcao MODBUS = 3)
    if ((CRC_OK) && (MsgRxS1[0] == 1) && (MsgRxS1[1] == 3)) {  // Se a mensagem recebida esta OK,
      Comando = MsgRxS1[2];                                    // Carrega o Comando
      //Serial.println("Id OK");
      ExecutaComando(Comando);
        
      // Carrega na mensagem a ser enviada as informacoes                               
      MsgTxS1[0] = 1;                  // Endereco da UTR Arduino Mega
      MsgTxS1[1] = 3;                  // Funcao
      MsgTxS1[2] = Comando;            // Comando Recebido do Concentrador Arduino Mega 
      MsgTxS1[3] = 0;                  // Reserva
      MsgTxS1[4] = rtc.getHour();      // Hora do RTC da UTR
      MsgTxS1[5] = rtc.getMinute();    // Minuto do RTC da UTR
      MsgTxS1[6] = rtc.getSecond();    // Segundo do RTC da UTR
      MsgTxS1[7] = rtc.getDay();       // Dia do RTC da UTR
      MsgTxS1[8] = rtc.getMonth();     // Mês do RTC da UTR
      MsgTxS1[9] = rtc.getYear();      // Ano do RTC da UTR (00 a 99)
      MsgTxS1[10] = EstadoCxAz;        // Estado da Caixa Azul
      
      MontaEDs();                      // Monta o Buffer EDbyte[] de 8 Bytes com 64 EDs Compactadas

      // Carrega os 8 Bytes com as 64 EDs no Buffer de Transmissao
      MsgTxS1[12] = EDbyte[0];
      MsgTxS1[13] = EDbyte[1];
      MsgTxS1[14] = EDbyte[2];
      MsgTxS1[15] = EDbyte[3];
      MsgTxS1[16] = EDbyte[4];
      MsgTxS1[17] = EDbyte[5];
      MsgTxS1[18] = EDbyte[6];
      MsgTxS1[19] = EDbyte[7];

      // Carrega as EAs no Buffer de Transmissao
      MsgTxS1[20] = ByteL(EA[0]);
      MsgTxS1[21] = ByteH(EA[0]);
      MsgTxS1[22] = ByteL(EA[1]);
      MsgTxS1[23] = ByteH(EA[1]);
      MsgTxS1[24] = ByteL(EA[2]);
      MsgTxS1[25] = ByteH(EA[2]);
      MsgTxS1[26] = ByteL(EA[3]);
      MsgTxS1[27] = ByteH(EA[3]);
      MsgTxS1[28] = ByteL(EA[4]);
      MsgTxS1[29] = ByteH(EA[4]);
      MsgTxS1[30] = ByteL(EA[5]);
      MsgTxS1[31] = ByteH(EA[5]);
      MsgTxS1[32] = ByteL(EA[6]);
      MsgTxS1[33] = ByteH(EA[6]);
      MsgTxS1[34] = ByteL(EA[7]);
      MsgTxS1[35] = ByteH(EA[7]);
      MsgTxS1[36] = ByteL(EA[8]);
      MsgTxS1[37] = ByteH(EA[8]);
      MsgTxS1[38] = ByteL(EA[9]);
      MsgTxS1[39] = ByteH(EA[9]);
      MsgTxS1[40] = ByteL(EA[10]);
      MsgTxS1[41] = ByteH(EA[10]);
      MsgTxS1[42] = ByteL(EA[11]);
      MsgTxS1[43] = ByteH(EA[11]);
      MsgTxS1[44] = ByteL(EA[12]);
      MsgTxS1[45] = ByteH(EA[12]);
      MsgTxS1[46] = ByteL(EA[13]);
      MsgTxS1[47] = ByteH(EA[13]);
      MsgTxS1[48] = ByteL(EA[14]);
      MsgTxS1[49] = ByteH(EA[14]);
      MsgTxS1[50] = ByteL(EA[15]);
      MsgTxS1[51] = ByteH(EA[15]);
      MsgTxS1[52] = ByteL(EAME0);
      MsgTxS1[53] = ByteH(EAME0);
      MsgTxS1[54] = ByteL(TmpBombaLig);
      MsgTxS1[55] = ByteH(TmpBombaLig);
      MsgTxS1[56] = ByteL(TmpCxAzNvBx);
      MsgTxS1[57] = ByteH(TmpCxAzNvBx);
        
      // Calcula o CRC16 e carrega nos ultimos dois bytes da mensagem
      for (int j = 0; j < (NBMsgT - 2); j++) {
        CRC = CRC ^ MsgTxS1[j];
        for (int i = 0; i < 8; i++) {
          lsb = CRC & 0x0001;
          if (lsb == 0){
            CRC = CRC / 2;
          }
          if (lsb == 1) {
            CRC = CRC / 2;
            CRC = CRC^0xA001;
          }
        }
      }
      MsgTxS1[NBMsgT - 2] = ByteL(CRC);
      MsgTxS1[NBMsgT - 1] = ByteH(CRC);

      //Serial.println("Tx Msg Resposta");
      //Serial.println("");
      
      // Transmite a Mensagem de Resposta para o Concentrador Arduino Mega pela Serial1
      delay(10);
      for (int i = 0; i < NBMsgT ; i++) {
        while (Serial1.availableForWrite() == 0) {
        }
        Serial1.write(MsgTxS1[i]);    // Envia o byte para o buffer de transmissao
        delayMicroseconds(100);
      }
    }
  }
} // Fim da Rotina ComunicacaoSerial()


//********************************************************************************************************************
// Nome da Rotina: MontaEDs()                                                                                         *
//                                                                                                                    *
// Funcao: monta os bytes de EDs para a mensagem de transmissao pela Interface Serial1 em Protocolo MODBUS            *
//                                                                                                                    *
// Entrada: nenhum                                                                                                    *
// Saida: nenhum                                                                                                      *
//*********************************************************************************************************************
//
void MontaEDs() {
  
  EDbyte[0] = digitalRead(DJEINV1);
  EDbyte[0] = (EDbyte[0] | (digitalRead(CircuitoBoia) << 1));
  EDbyte[0] = (EDbyte[0] | (digitalRead(BoiaCxAzul) << 2));
  EDbyte[0] = (EDbyte[0] | (digitalRead(CircuitoBomba) << 3));
  EDbyte[0] = (EDbyte[0] | (digitalRead(AlimRedeBomba) << 4));
  EDbyte[0] = (EDbyte[0] | (CxAzNivBaixo << 5));
  EDbyte[0] = (EDbyte[0] | (EstadoRede << 6));
  EDbyte[0] = (EDbyte[0] | (ModoOperacao << 7));

  EDbyte[1] = ModoComando;
  EDbyte[1] = EDbyte[1] | (ModoControle << 1);
  EDbyte[1] = EDbyte[1] | (Carga1 << 2);
  EDbyte[1] = EDbyte[1] | (Carga2 << 3);
  EDbyte[1] = EDbyte[1] | (Carga3 << 4);
  EDbyte[1] = EDbyte[1] | (Carga4 << 5);
  EDbyte[1] = EDbyte[1] | (HabCom << 6);
  EDbyte[1] = EDbyte[1] | (EstadoInversor1 << 7);

  EDbyte[2] = ModoComando;
  EDbyte[2] = EDbyte[2] | (EstadoInversor2 << 1);
  EDbyte[2] = EDbyte[2] | (EstadoCarga3 << 2);
  EDbyte[2] = EDbyte[2] | (BombaLigada << 3);
  EDbyte[2] = EDbyte[2] | (FlgTmpFT << 4);
  EDbyte[2] = EDbyte[2] | (CTRF << 5);
  EDbyte[2] = EDbyte[2] | (FlgTmpFTR << 6);
  EDbyte[2] = EDbyte[2] | (ModoControle1 << 7);

  EDbyte[3] = FalhaInversor1;
  EDbyte[3] = EDbyte[3] | (SubTensaoInv1<< 1);
  EDbyte[3] = EDbyte[3] | (SobreTensaoInv1 << 2);
  EDbyte[3] = EDbyte[3] | (SobreTempDrInv1 << 3);
  EDbyte[3] = EDbyte[3] | (SobreTempTrInv1 << 4);
  EDbyte[3] = EDbyte[3] | (DisjAbertoIv1 << 5);
  EDbyte[3] = EDbyte[3] | (FalhaInversor2 << 6);
  EDbyte[3] = EDbyte[3] | (SubTensaoInv2 << 7);

  EDbyte[4] = SobreTensaoInv2;
  EDbyte[4] = EDbyte[4] | (SobreTempDrInv2 << 1);
  EDbyte[4] = EDbyte[4] | (SobreTempTrInv2 << 2);
  EDbyte[4] = EDbyte[4] | (DisjAbertoIv2 << 3);
  EDbyte[4] = EDbyte[4] | (digitalRead(SD00) << 4);
  EDbyte[4] = EDbyte[4] | (digitalRead(SD01) << 5);
  EDbyte[4] = EDbyte[4] | (digitalRead(SD02) << 6);
  EDbyte[4] = EDbyte[4] | (digitalRead(SD03) << 7);

  EDbyte[5] = digitalRead(SD04);
  EDbyte[5] = EDbyte[5] | (digitalRead(SD05) << 1);
  EDbyte[5] = EDbyte[5] | (digitalRead(SD06) << 2);
  EDbyte[5] = EDbyte[5] | (digitalRead(SD07) << 3);
  EDbyte[5] = EDbyte[5] | (digitalRead(SD08) << 4);
  EDbyte[5] = EDbyte[5] | (digitalRead(SD09) << 5);
  EDbyte[5] = EDbyte[5] | (digitalRead(SD10) << 6);
  EDbyte[5] = EDbyte[5] | (digitalRead(SD11) << 7);

  EDbyte[6] = digitalRead(SD12);
  EDbyte[6] = EDbyte[6] | (digitalRead(SD13) << 1);
  EDbyte[6] = EDbyte[6] | (digitalRead(SD14) << 2);
  EDbyte[6] = EDbyte[6] | (digitalRead(SD15) << 3);
  EDbyte[6] = EDbyte[6] | (digitalRead(SD16) << 4);
  EDbyte[6] = EDbyte[6] | (digitalRead(SD17) << 5);
  EDbyte[6] = EDbyte[6] | (digitalRead(EdCxAzCheia) << 6);
  EDbyte[6] = EDbyte[6] | (digitalRead(FonteCC2Lig) << 7);
  
  EDbyte[7] = digitalRead(FonteCC1Lig);
  EDbyte[7] = EDbyte[7] | (SobreCorrenteInv1 << 1);
  EDbyte[7] = EDbyte[7] | (SobreCorrenteInv2 << 2);
  
} // Fim da Rotina MontaEDs


//*********************************************************************************************************************
// Nome da Rotina: ExecutaComando                                                                                     *
//                                                                                                                    *
// Funcao: executa um comando                                                                                         *
//                                                                                                                    *
// Entrada: codigo de identificacao do comando e habilitacao de impressao de mensagem                                 *
// Saida: nenhum                                                                                                      *
//*********************************************************************************************************************
//
void ExecutaComando(byte codigo) {

  switch (codigo) {
   
    case 1: // Resposta a Solicitacao de Leitura de Estados
      
    break;

    case 2: // Comando de Acerto de Relogio
      HoraRec = MsgRxS1[4];
      MinutoRec = MsgRxS1[5];
      SegundoRec = MsgRxS1[6];
      DiaRec = MsgRxS1[7];
      MesRec = MsgRxS1[8];
      AnoRec = MsgRxS1[9];
    
      tm.Hour = HoraRec;
      tm.Minute = MinutoRec;
      tm.Second = SegundoRec;
      tm.Day = DiaRec;
      tm.Month = MesRec;
      tm.Year = AnoRec;

      rtc.setDate(tm.Day, 6, tm.Month, 0, tm.Year);
      rtc.setTime(tm.Hour, tm.Minute, tm.Second);
   
    break;
  
    case 3: // Comando de Modo de Operacao 0 - Economia de Bateria
      if (ModoComando == 1) { // Verifica se esta em modo Remoto 
        ModoOperacao = 0;
      }
    break;
  
    case 4: // Comando de Modo de Operacao 1 - Normal
      if (ModoComando == 1) { // Verifica se esta em modo Remoto
        ModoOperacao = 1;
      }
    break;
  
    case 5: // Comando 5: Modo de Controle = 0 => Carga 3 desabilitada na falta de CA
      if (ModoComando == 1) {
        ModoControle = 0;
        Serial.print("Comando = 5 - ModoControle = ");
        Serial.println(ModoControle);
      } 
    break;
    
    case 6: // Comando 6: Modo de Controle = 1 => Carga 3 habilitada na falta de CA
      if (ModoComando == 1) {
        ModoControle = 1;
        Serial.print("Comando = 6 - ModoControle = ");
        Serial.println(ModoControle);
      }
    break;

    case 7: // Comando de Modo de Habilita Carga1
      if (ModoComando == 1) { // Verifica se esta em modo Remoto
        Carga1 = 1;
      }
    break;

    case 8: // Comando de Modo de Desabilita Carga1
      if (ModoComando == 1) { // Verifica se esta em modo Remoto
        Carga1 = 0;
      }
    break;

    case 9: // Comando de Modo de Habilita Carga2
      if (ModoComando == 1) { // Verifica se esta em modo Remoto
        Carga2 = 1;
      }
    break;

    case 10: // Comando de Modo de Desabilita Carga2
      if (ModoComando == 1) { // Verifica se esta em modo Remoto
        Carga2 = 0;
      }
    break;
  
    case 11: // Comando de Modo de Habilita Carga3
      if (ModoComando == 1) { // Verifica se esta em modo Remoto
        Carga3 = 1;
      }
    break;

    case 12: // Comando de Modo de Desabilita Carga3
      if (ModoComando == 1) { // Verifica se esta em modo Remoto
        Carga3 = 0;
      }
    break;

    case 13: // Comando de Modo de Habilita Carga4
      if (ModoComando == 1) { // Verifica se esta em modo Remoto
        Carga4 = 1;
      }
    break;

    case 14: // Comando de Modo de Desabilita Carga4
      if (ModoComando == 1) { // Verifica se esta em modo Remoto
        Carga4 = 0;
      }
    break;

    case 15: // Comando de Limpeza dos Indicadores de Falha
      if (ModoComando == 1) {
        FalhaInversor1 = 0;
        DisjAbertoIv1 = 0;
        SubTensaoInv1 = 0;
        SobreTensaoInv1 = 0;
        SobreTempDrInv1 = 0;
        SobreTempTrInv1 = 0;
        FalhaInversor2 = 0;
        DisjAbertoIv2 = 0;
        SubTensaoInv2 = 0;
        SobreTensaoInv2 = 0;
        SobreTempDrInv2 = 0;
        SobreTempTrInv2 = 0;
        FalhaCarga3 = 0;
      }
    break;

    case 16: // Comando de Modo Manual para a Carga 1 
      if (ModoComando == 1) {
        ModoControle1 = 0;
        Serial.println("Recebido comando Modo Manual Carga 1");
      } 
    break;

    case 17: // Comando de Modo Automatico para a Carga 1
      if (ModoComando == 1) {
        ModoControle1 = 1;
      }
    break;

    case 18: // Comando de Modo OnGrid 
      if (ModoComando == 1) {
        OpOffGrid = false;
      } 
    break;

    case 19: // Comando de Modo OffGrid 
      if (ModoComando == 1) {
        OpOffGrid = true;
      } 
    break;

    } // switch (codigo)
  
} // Fim da rotina ExecutaComando


//*********************************************************************************************************************
// Nome da Rotina: CarregaMedidas                                                                                     *
//                                                                                                                    *
// Funcao: carrega as medidas a partir das entradas analogicas e faz a media com dez medidas                          *
//                                                                                                                    *
// Entrada: nenhum                                                                                                    *
// Saida: array EA[] contem as medias das 16 medidas das entradas analogicas                                          *
//*********************************************************************************************************************
//
void CarregaMedidas() {
  
  // Soma nos Registros de Acumulacao para Calculo de Media
  EAT[0] = EAT[0] + analogRead(EA00);
  EAT[1] = EAT[1] + analogRead(EA01);
  EAT[2] = EAT[2] + analogRead(EA02);
  EAT[3] = EAT[3] + analogRead(EA03);
  EAT[4] = EAT[4] + analogRead(EA04);
  EAT[5] = EAT[5] + analogRead(EA05);
  EAT[6] = EAT[6] + analogRead(EA06);
  EAT[7] = EAT[7] + analogRead(EA07);
  EAT[8] = EAT[8] + analogRead(EA08);
  EAT[9] = EAT[9] + analogRead(EA09);
  EAT[10] = EAT[10] + analogRead(EA10);
  EAT[11] = EAT[11] + analogRead(EA11);
  EAT[12] = EAT[12] + analogRead(EA12);
  EAT[13] = EAT[13] + analogRead(EA13);
  EAT[14] = EAT[14] + analogRead(EA14);
  EAT[15] = EAT[15] + analogRead(EA15);
  
  CTEA = CTEA + 1;                  // Incrementa o contador de medidas
  if (CTEA == NCCMEA) {             // Se terminou o numero de ciclos de calculo de media das entradas analogicas
    CTEA = 0;                       // Zera o contador de media

    EA[0] = KMed00 * ((EAT[0] / NCCMEA) - Offset00);
    EA[1] = KMed01 * ((EAT[1] / NCCMEA) - Offset01);
    EA[2] = KMed02 * ((EAT[2] / NCCMEA) - Offset02);
    EA[3] = KMed03 * ((EAT[3] / NCCMEA) - Offset03);
    EA[4] = KMed04 * ((EAT[4] / NCCMEA) - Offset04);
    EA[5] = KMed05 * ((EAT[5] / NCCMEA) - Offset05);
    EA[6] = KMed06 * ((EAT[6] / NCCMEA) - Offset06);
    EA[7] = KMed07 * ((EAT[7] / NCCMEA) - Offset07);
    EA[8] = KMed08 * ((EAT[8] / NCCMEA) - Offset08);
    EA[9] = KMed09 * ((EAT[9] / NCCMEA) - Offset09);
    EA[10] = KMed10 * ((EAT[10] / NCCMEA) - Offset10);
    EA[11] = KMed11 * ((EAT[11] / NCCMEA) - Offset11);
    EA[12] = KMed12 * ((EAT[12] / NCCMEA) - Offset12);
    EA[13] = KMed13 * ((EAT[13] / NCCMEA) - Offset13);
    EA[14] = KMed14 * ((EAT[14] / NCCMEA) - Offset14);
    EA[15] = KMed15 * ((EAT[15] / NCCMEA) - Offset15);
    
    // Zera os acumuladores de media e as medidas que forem negativas
    for (int i = 0; i < 16; i++) {
      EAT[i] = 0;                   
      if (EA[i] < 0) {
        EA[i] = 0;
      }
    }

    VBat = EA[0];         // EA00 - Tensao CC: Barramento Principal de 24 Volts
    ICirCC = EA[3];       // EA03 - Corrente CC: Circuitos de Corrente Continua
    VRede = EA[5];        // EA05 - Tensao CA: Rede

    IEInv1 = EA[15];      // EA15 - Corrente CC: Entrada do Inversor 1 (ok)    
    VSInv1 = EA[4];       // EA04 - Tensao CA: Saida Inversor 1 (ok)
    ISInv1 = EA[13];      // EA13 - Corrente CA: Saida do Inversor 1 (ok)
    TDInv1 = EA[2];       // EA02 - Temperatura: Driver do Inversor 1 (ok)
    TTInv1 = EA[7];       // EA07 - Temperatura: Transformador do Inversor 1 (ok)

    IEInv2 = EA[12];      // EA12 - Corrente CC: Entrada do Inversor 2 (ok)
    VSInv2 = EA[6];       // EA06 - Tensao CA: Saida Inversor 2 (ok)
    ISInv2 = EA[10];      // EA10 - Corrente CA: Saida do Inversor 2 (ok)
    TDInv2 = EA[8];       // EA08 - Temperatura: Drivers do Inversor 2 (ok)
    TTInv2 = EA[9];       // EA09 - Temperatura: Transformador do Inversor 2 (ok)
    
    ISFtcc = EA[11];      // EA11 - Corrente CC: Saida da Fonte CC
    IEInv1 = EA[12];      // EA12 - Corrente CC: Entrada do Inversor 1
    ICg3 = EA[14];        // EA14 - Corrente CA: Carga 3
  
    // Calculo da Media Estendida da Tensao 24Vcc do Barramento
    EAMET0 = EAMET0 + EA[0];
    CTME = CTME + 1;
    if (CTME == NCMED) {              // Se foram feitos os ciclos de media de aquisicao analogica estendida,
      EAME0 = EAMET0 / NCMED;         // calcula a media estendida da tensao CC no periodo definido
      EAMET0 = 0;                     // zera o acumulador para calculo de media estendida
      CTME = 0;                       // zera o contador de media estendida
    }

  } // if (CTEA == NCCMEA)

} // Fim da Rotina


//*********************************************************************************************************************
// Nome da Rotina: VerificaTensaoRede                                                                                 *
//                                                                                                                    *
// Funcao: atualiza a variavel EstadoRede de acordo com a tensao da rede                                              *
//                                                                                                                    *
// Entrada: EA[5] = tensao da rede                                                                                    *
// Saida: variavel byte EstadoRede (0 = tensao baixa) / (1 = normal)                                                  *
//*********************************************************************************************************************
//
void VerificaTensaoRede() {

  int Hr = rtc.getHour();

  if (VRede < VTRNFCA) {            // Se a tensao da rede cair abaixo do limite inferior,
    if (CTFCA > 0) {                // e se esta temporizando para confirmar Falta de CA,
      CTFCA = CTFCA - 1;            // decrementa contador
    }
    else {                          // Se o contador de temporizacao chegou a zero, confirma a Falta CA
      EstadoRede = 0;               // Sinaliza Falta CA na rede,
      EvtFaltaCA = true;            // ativa o indicador de falta de CA na rede,
      CTRCA = 0;                    // e zera o contador de temporizacao para verificacao de retorno de tensao da rede
    }
  }
  
  if (EstadoRede == 0) {
    if (CTEA == 0) {                // Se terminou um ciclo de media de entradas analogicas,
      if (VRede > VTRFNCA) {        // Se a tensao da rede esta maior que o limite superior (voltou ao normal),
        CTRCA = CTRCA + 1;          // incrementa o contador de temporizacao
        if (CTRCA >= TRRede) {      // Se a tensao da rede se manteve acima do limite pelo tempo especificado,
          EstadoRede = 1;           // sinaliza que a tensao da rede esta OK.
          CTRCA = 0;                // Zera o contador de temporizacao
          CTFCA = TFRede;           // Carrega o valor de temporizacao para detecao de falta de CA
        }
      }
      else {                        // Se a tensao da rede caiu abaixo do limite durante a temporizacao,
        CTRCA = 0;                  // zera o contador de temporizacao para iniciar novamente
      }
    } // if (CTEA == 0)
  } // if (EstadoRede == 0)
  
} // Fim da Rotina


//*********************************************************************************************************************
// Nome da Rotina: VerificaCarga3                                                                                     *
//                                                                                                                    *
// Funcao: atualiza a variavel EstadoCarga3 de acordo com a corrente da Carga 3                                       *
//                                                                                                                    *
// Entrada: EA[15] = corrente da carga 3 em mA                                                                        *
// Saida: variavel byte EstadoCarga3 (0 = desligada) / (1 = ligada)                                                   *
//*********************************************************************************************************************
//
void VerificaCarga3() {

  if (ICg3 < LIMICC3) {         // Se a corrente da Carga 3 cair abaixo de 100mA
    EstadoCarga3 = 0;           // Atualiza o estado para desligado,
  }

  if ((ICg3 >= LIMSCC3) && (ICg3 <= LIMFCC3)) {  // Se a corrente da Carga 3 > 400mA e menor que 800mA
    EstadoCarga3 = 1;           // Atualiza o estado para ligado
  }

  if (digitalRead(SD02) == HIGH) {  // Se a Carga 3 esta para o Inversor,
    if (ICg3 > LIMFCC3) {           // e se a corrente da Carga 3 > 800mA
      FalhaCarga3 = 1;              // Aciona o indicador de falha,
      Carga3 = 0;                   // e desabilita a Carga 3
    }
  }
}

//*********************************************************************************************************************
// Nome da Rotina: VerificaChaves()                                                                                   *
//                                                                                                                    *
// Funcao: le o estado das chaves de Selecao de Modo de Operacao e Prioridade e carrega nas variaveis                 *
//                                                                                                                    *
// Entrada: leitura das chaves nas EDs                                                                                *
//          ChLR => Cima = Remoto = 1 / Baixo = Local = 0                                                             *
//          ChEN => Cima = Normal = 1 / Baixo = Economia de Bateria = 0)                                              *
//          ChMA => Cima = Automatico = 1 / Manual = 0                                                                *
//          ChPrB => Posicao Baixo = 1                                                                                *
//          ChPrC => Posicao Cima = 1                                                                                 *
//                                                                                                                    *
// Saida: variaveis ModoOperacao e ModoControle                                                                       *
//*********************************************************************************************************************
//
void VerificaChaves() {

  ModoComando = digitalRead(ChLR);     // carrega o estado da chave Local = 0 / Remoto = 1
  
  if (ModoComando == 0) {              // Se o Modo de Comando = Local ou vem do reset,
    
    ModoOperacao = digitalRead(ChEN);  // carrega o estado da chave Modo de Operacao: Normal = 1 / Economia = 0
    OpOffGrid = digitalRead(ChOnOff);  // carrega o estado da chave Modo OffGrid = 1 / Modo OnGrid = 0
    
    //ModoControle1 = ModoControle;
  
    // se a chave de Habilitacao de Cargas esta para baixo, desabilita a Carga3 na falta de CA
    if ((digitalRead(ChPrB) == 1) && (digitalRead(ChPrC) == 0)) {
      ModoOperacao = 1;
      Carga1 = 0;
      Carga2 = 0;
      Carga3 = 0;
      ModoControle = 0;
      Carga4 = 0;
    }

    // se a chave de Habilitacao de Cargas esta no meio usa a configuração padrão
    if ((digitalRead(ChPrB) == 0) && (digitalRead(ChPrC) == 0)) {
      ModoOperacao = 1;
      Carga1 = 0;
      Carga2 = 0;
      Carga3 = 0;
      ModoControle = 1;
      Carga4 = 0;
    }

    // se a chave de Habilitacao de Cargas esta para cima, passa a Carga2, Carga3 e Carga1 para o Inversor 1
    if ((digitalRead(ChPrB) == 0) && (digitalRead(ChPrC) == 1)) {
      ModoOperacao = 1;
      Carga1 = 1;
      Carga2 = 1;
      Carga3 = 1;
      ModoControle = 1;
      Carga4 = 0;
    }
  }
  
} // Fim da Rotina VerificaChaves()


//*************************************************************************************************************
// Nome da Rotina: VerificaModoOp()                                                                           *
//                                                                                                            *
// Funcao: verifica qual Modo de Operacao deve ser selecionado                                                *
//                                                                                                            *
// Entrada: Estado24Vcc, ModoOperacao, ModoControle                                                           *
// Saida: não tem                                                                                             *
//*************************************************************************************************************
//
void VerificaModoOp() {

  int HrMn = 100 * (rtc.getHour()) + rtc.getMinute();

  if (OpOffGrid) {                         // Se é operação Off Grid,
    if ((HrMn > HHC1) && (HrMn < HDC1)) {  // e se o horário tem Sol,
        digitalWrite(SD16,1);              // aciona os relés e conecta os paineis solares nos Controladores de Carga
    }
    else {                                 // Se o horário não tem Sol,
      digitalWrite(SD16,0);                // desativa os relés
    }
  }
  else {                                     // Se não é operação Off Grid,
    if (EstadoRede == 0) {                   // e se há falta de CA,
      if ((HrMn > HHC1) && (HrMn < HDC1)) {  // e se o horário tem Sol,
        digitalWrite(SD16,1);                // aciona os relés e conecta os paineis solares nos Controladores de Carga
      }
      else {                                 // Se o horário não tem Sol,
        digitalWrite(SD16,0);                // desativa os relés  
      }
    }
    else {                                   // Se a tensão CA está normal,
      digitalWrite(SD16,0);                  // desativa os relés e conecta os paineis solares no Inversor On Grid
    }
  }
  
  
  if ((ModoOperacao == 0) || (EAME0 < VMinBat)) {  // Se Modo de Operacao = Economia ou Tensão 24Vcc está baixa,
    CT1_Rede();                                    // passa CT1 para a Rede (Carga 2 - Torre)
    CT3_Rede();                                    // passa CT3 para a Rede (Carga 3 - Portao e Iluminacao Externa)
    CT2_Rede();                                    // passa CT2 para a Rede (Carga 1 - Casa Verde e Oficina)
    DesligaInversor2();                            // e desliga o Inversor 1
  }
  else {                               // Se o Modo de Operacao = Normal e a tensão 24Vcc está maior que o mínimo
    if (EstadoRede == 1) {             // e se a Tensao da Rede esta OK,
      if (OpOnGrid) {                  // e se é o modo de operação On Grid,
            
        if (Carga2 == 1) { 
          if (FalhaInversor2 == 0) {
            if (EAME0 > 2350) {
              LigaInversor2();
              CT1_Inversor();
            }
          }
        }
        else {
          CT1_Rede();
          if (Carga3 == 1) {
            Carga3 = 0;
            CT3_Rede();
          }
          if (Carga1 == 1) {
            Carga1 = 0;
            CT2_Rede();
          }
          if (EstadoInversor2 == 1) {
            DesligaInversor2();
          }
        }

        if (Carga3 == 1) {
          if (EstadoInversor2 == 1) {
            if (EAME0 > 2500) {
              CT3_Inversor();
            }
          }
        }
        else {                                   // Se a Carga3 está desabilitada,
          CT3_Rede();                            // passa a Carga3 para a Rede,
        }

        if (Carga1 == 1) {
          if (EstadoInversor2 == 1) {
            if (EAME0 > 2550) {
              CT2_Inversor();
            }
          }
        }
        else {                                   // Se a Carga1 está desabilitada,
          CT2_Rede();                            // passa CT2 para a Rede
        }
        
      } // if (OpOnGrid)
    } // if (EstadoRede == 1)

    //*****************************************************************************************************
    //                                Se há falta de tensão na Rede                                       *
    //*****************************************************************************************************
    //
    if (EstadoRede == 0) {
      
      if (FalhaInversor2 == 0) {   // Se nao ha falha no Inversor 1,
        if (EAME0 > 2350) {        // e se a Tensão 24Vcc está maior que o mínimo
          LigaInversor2();         // Liga o Inversor 1,
          CT1_Inversor();          // e passa CT1 para o Inversor 1 (Carga 2 => Torre)
        }
      }

      if ((Carga3 == 1) || (ModoControle == 1)) {  // Se a Carga3 está no Inversor 1 ou habilitada na falta de CA,
        if (EstadoInversor2 == 1) {  // e se o Inversor 1 está ligado,
          if (EAME0 > 2500) {        // e se a tensão 24Vcc permite,
            CT3_Inversor();          // passa CT3 para o Inversor 1
          }
          if (EAME0 < 2400) {        // Se a tensão 24Vcc está abaixo do mínimo para esta carga,
            CT3_Rede();              // passa CT3 para a rede
          }
        }
        else {
          CT3_Rede();                // Se o Inversor 1 está desligado, passa CT3 para a rede
        }
      }
      else {        // se a Carga3 não está no Inversor 1 e não está habilitada na falta de CA, passa CT3 para a rede
        CT3_Rede();
      }
      
      if (Carga1 == 1) {
        if (EstadoInversor2 == 1) {
          if (EAME0 > 2550) {
            CT2_Inversor();
          }
          if (EAME0 < 2450) {
            CT2_Rede();
          }
        }
        else {
          CT2_Rede();
        }
      }
      else {
        CT2_Rede();
      }
  
    } // if (EstadoRede == 0)
  } // else if ((ModoOperacao == 0) || (EAME0 < VMinBat))
  
}


//*********************************************************************************************************************
// Nome da Rotina: VerificaInversor2                                                                                  *
//                                                                                                                    *
// Funcao: verifica o funcionamento do Inversor 2, liga os indicadores de alarme e aciona os LEDs                     *
//                                                                                                                    *
// Entrada: SD10 => (0 = Inversor Desligado) / (1 = Inversor Ligado)                                                  *
//          SD11 => (0 = Inversor Conectado na CT4) / (1 = Inversor Desconectado da CT4)                              *
//          EA[6] = tensao CA de saida do Inversor 2, variavel EstadoRede                                             *
//          EA[15] = temperatura do driver, EA[9] = temperatura do transformador                                      *                          *
//                                                                                                                    *
// Saida: variaveis de alarme                                                                                         *            *
//*********************************************************************************************************************
//
void VerificaInversor2() {

  if (digitalRead(SD10) == HIGH) {    // Se o Circuito de Controle do Inversor 2 esta ligado,
    if (digitalRead(DJEINV2) == 1) {  // se o disjuntor de entrada 24Vcc esta ligado,
      
      if (VSInv2 >= VMinInv2) {       // e se a saida 220VCA esta com a tensao normal:
        if (VSInv2 < VMaxInv2) {
          EstadoInversor2 = 1;        // Sinaliza Inversor 2 Ligado e Funcionando
          CTIV2 = 0;                  // Zera o contador de tempo de espera para o inversor ligar
        }
      }
      if (VSInv2 < VMinInv2) {        // Se a tensao de saida do Inversor 2 esta baixa,
        CTIV2 = CTIV2 + 1;            // incrementa o contador de espera para o inversor ligar.
        if (CTIV2 > CTRI) {           // Se o inversor nao voltar ao normal em 5 segundos:
          CTIV2 = 0;                  //   1) Zera o contador de espera
          FalhaInversor2 = 1;         //   2) Liga alarme sinalizando ocorrencia de falha no inversor
          SubTensaoInv2 = 1;          //   3) Liga o indicador que indica subtensao na saida do inversor
        }
      }
      if (VSInv2 >= VMaxInv2) {       // Se a tensao de saida do Inversor 2 esta alta,
        FalhaInversor2 = 1;           // Liga alarme sinalizando ocorrencia de falha no inversor
        SobreTensaoInv2 = 1;          // Liga o indicador de sobretensao na saida do inversor
      }
      
      if (TDInv2 >= LiTDInv) {        // Caso a temperatura do driver do Inversor 2 esta alta,
        FalhaInversor2 = 1;           // liga alarme sinalizando ocorrencia de falha no inversor,
        SobreTempDrInv2 = 1;          // liga o indicador que indica sobretemperatura no transformador do inversor,
      }
      
      if (TTInv2 >= LiTTInv) {        // Caso a temperatura do transformador do Inversor 2 esta alta,
        FalhaInversor2 = 1;           // liga alarme sinalizando ocorrencia de falha no inversor,
        SobreTempTrInv2 = 1;          // liga o indicador que indica sobrecorrente no transformador do inversor,
      }

      if (IEInv2 >= IMaxInv2) {       // Se a corrente de entrada do Inversor 2 esta alta,
        FalhaInversor2 = 1;           // liga alarme sinalizando ocorrencia de falha no inversor,
        SobreCorrenteInv2 = 1;        // liga o indicador que indica sobrecorrente no transformador do inversor,
      }
    } // if (digitalRead(DJEINV2)
    else {                            // Se o disjuntor de entrada 24Vcc esta aberto,
      //FalhaInversor2 = 1;           // liga alarme sinalizando ocorrencia de falha no inversor,
      //DisjAbertoIv2 = 1;            // liga indicador identificando a falha
    }

    if (FalhaInversor2 == 1) {
      DesligaInversor2();
      EstadoInversor2 = 0;            // Sinaliza Inversor 2 desligado
    }
  }
  else {                              // Se o Inversor 2 esta desligado,
    EstadoInversor2 = 0;              // sinaliza Inversor 2 desligado
  }
  
}


//*********************************************************************************************************************
// Nome da Rotina: LigaInversor2()                                                                                    *
//                                                                                                                    *
// Funcao: liga o Inversor 2 e passa a Chave de Transferencia 4 para o Inversor 2                                     *
//                                                                                                                    *
// Entradas: nenhuma                                                                                                  *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void LigaInversor2() {

  if (FalhaInversor2 == 0) {          // Se o Inversor 2 nao esta em falha,
    if (digitalRead(SD10) == LOW) {   // e se o Inversor 2 esta desligado,
      digitalWrite(SD10, HIGH);       // Liga o Inversor 2
    }
  }
  
}


//*********************************************************************************************************************
// Nome da Rotina: DesligaInversor2()                                                                                 *
//                                                                                                                    *
// Funcao: desliga o Inversor 2                                                                                       *
//                                                                                                                    *
// Entradas: nenhuma                                                                                                  *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void DesligaInversor2() {
  
  if (digitalRead(SD10) == HIGH) {  // Se o Inversor 2 esta ligado,
    digitalWrite(SD10, LOW);        // Desliga o Inversor 2
  }
  
}


//*********************************************************************************************************************
// Nome da Rotina: CT2_Rede()                                                                                         *
//                                                                                                                    *
// Funcao: passa CT2 (Carga1 - Casa Verde e Oficina) para a Rede                                                      *
//                                                                                                                    *
// Entradas: nenhuma                                                                                                  *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void CT2_Rede() {

  if (digitalRead(SD17) == HIGH) {  // Se a CT2 esta para o Inversor 2,
    digitalWrite(SD17, LOW);        // passa a CT2 (Carga1 - Casa Verde e Oficina) para a rede
  }

} // Fim da Rotina CT2_Rede()


//*********************************************************************************************************************
// Nome da Rotina: CT2_Inversor()                                                                                     *
//                                                                                                                    *
// Funcao: passa CT2 (Carga1 - Casa Verde e Oficina) para o Inversor 2                                                *
//                                                                                                                    *
// Entradas: nenhuma                                                                                                  *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void CT2_Inversor() {

  if (digitalRead(SD17) == LOW) {    // Se CT2 esta para a rede,
      if (EstadoInversor2 == 1) {    // e se o Inversor 2 esta ligado e funcionando,
        digitalWrite(SD17, HIGH);    // passa CT2 para o Inversor 1
      }
  }
 
}


//*********************************************************************************************************************
// Nome da Rotina: CT1-Inversor()                                                                                     *
//                                                                                                                    *
// Funcao: passa a Chave de CT1 (Carga2 - Torre) para o Inversor 2                                                    *
//                                                                                                                    *
// Entradas: nenhuma                                                                                                  *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void CT1_Inversor() {

  if (digitalRead(SD00) == LOW) {    // Se CT1 esta para a rede,
      if (EstadoInversor2 == 1) {    // e se o Inversor 2 esta ligado e funcionando,
        digitalWrite(SD00, HIGH);    // passa CT1 para o Inversor 2
      }
  }
  
}


//*********************************************************************************************************************
// Nome da Rotina: CT1-Rede()                                                                                         *
//                                                                                                                    *
// Funcao: passa a Chave CT1 (Carga2 - Torre) para a Rede                                                             *
// Entradas: nenhuma                                                                                                  *
//                                                                                                                    *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void CT1_Rede() {

  if (digitalRead(SD00) == HIGH) {  // Se CT1 esta para o Inversor 2,
    digitalWrite(SD00, LOW);        // Passa CT1 para a rede
  }
  
}


//*********************************************************************************************************************
// Nome da Rotina: CT3-Inversor()                                                                                     *
//                                                                                                                    *
// Funcao: passa CT3 (Carga3 - Portão, Câmeras da Entrada e Luzes da Entrada) para o Inversor 2                       *
// Entradas: nenhuma                                                                                                  *
//                                                                                                                    *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void CT3_Inversor() {

  if (digitalRead(SD02) == LOW) {    // Se CT3 esta para a rede,
      if (EstadoInversor2 == 1) {    // e se o Inversor 2 esta ligado e funcionando,
        digitalWrite(SD02, HIGH);    // passa CT3 para o Inversor 1
      }
  }
   
}


//*********************************************************************************************************************
// Nome da Rotina: CT3-Rede()                                                                                         *
//                                                                                                                    *
// Funcao: passa CT3 (Carga3 - Portão, Câmeras da Entrada e Luzes da Entrada) para a Rede                             *
//                                                                                                                    *
// Entradas: nenhuma                                                                                                  *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void CT3_Rede() {

  if (digitalRead(SD02) == HIGH) {  // Se CT3 esta para o Inversor 2,
    digitalWrite(SD02, LOW);        // Passa CT3 para a rede
  }
  
}


//*********************************************************************************************************************
// Nome da Rotina: VerifCxAzBomba()                                                                                   *
//                                                                                                                    *
// Funcao: verifica o estado da Caixa Azul e o funcionamento da Bomba do Poco                                         *
//                                                                                                                    *
// Entradas: nenhuma                                                                                                  *
//                                                                                                                    *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void VerifCxAzBomba() {

  if ((BombaLigada == 0) && (digitalRead(CircuitoBomba) == 1)) {  // se a Bomba ligou,
    if (CxAzCheia) {                                              // e se a Caixa Azul encheu antes,
      TmpBombaLig = 0;                                            // zera o temporizador de Bomba Ligada,
      CxAzCheia = false;                                          // e limpa o indicador de Caixa Azul Cheia
    }
    BombaLigada = 1;                                              // Atualiza o indicador de estado da bomba,
    HabTmpBombaLig = true;                                        // e habilita a contagem de tempo de bomba ligada
  }

  if ((BombaLigada == 1) && (digitalRead(CircuitoBomba) == 0)) {  // Se a Bomba desligou,
    BombaLigada = 0;                                              // atualiza o indicador de estado da bomba,
    HabTmpBombaLig = false;                                       // e desabilita a contagem de tempo de bomba ligada
  }

  if ((!Silencia) && (digitalRead(BotaoSilencia) == 1)) {
    Silencia = true;
  }

  if (CxAzPrecEncher) {                                  // Se a Caixa Azul precisa encher
    if (digitalRead(BoiaCxAzNivBx) == 1) {               // e se o Nivel esta baixo,
      CxAzNivBaixo = 1;                                  // Liga o alarme de nivel baixo
      HabTmpCxAzNvBx = true;                             // Habilita a contagem de tempo da caixa nivel baixo
      if (Silencia) {                                    // Se foi pressionado o botao de silenciar,
        digitalWrite(SD18, 0);                           // desliga a sirene.
      }
      else {                                             // se nao esta silenciada a sirene,
        digitalWrite(SD18, 1);                           // liga a sirene
      }
    }
    else {                                               // Se o nivel da caixa azul nao esta baixo,
      CxAzNivBaixo = 0;                                  // desliga o alarme de nivel baixo,
      digitalWrite(SD18, 0);                             // desliga a sirene,
      HabTmpCxAzNvBx = false;                            // desabilita a contagem de tempo da caixa nivel baixo
      Silencia = false;                                  // desliga a flag de silencia a sirene.
    }
  }
}


//*********************************************************************************************************************
// Nome da Rotina: VerifEstCxAz()                                                                                     *
//                                                                                                                    *
// Funcao: le as entradas digitais de sinalizacao dos estados da Caixa Azul                                           *
//                                                                                                                    *
// Entrada: nao tem                                                                                                   *
// Saidas:                                                                                                            *
//         EstadoCxAz = 0 => Estado da Caixa Azul = Indefinido                                                        *
//         EstadoCxAz = 1 => Estado da Caixa Azul = Precisa Encher Nivel Baixo                                        *
//         EstadoCxAz = 2 => Estado da Caixa Azul = Precisa Encher Nivel Normal                                       *
//         EstadoCxAz = 3 => Estado da Caixa Azul = Cheia                                                             *
//         EstadoCxAz = 4 => Estado da Caixa Azul = Falha de Sinalizacao 1                                            *
//         EstadoCxAz = 5 => Estado da Caixa Azul = Falha de Sinalizacao 2                                            *
//*********************************************************************************************************************
//
void VerifEstCxAz() {
  
  if (digitalRead(CircuitoBoia) == 1) {  // Se o circuito de sinalizacao da Caixa Azul esta ligado,
    
    if ((digitalRead(BoiaCxAzul) == 1) && digitalRead(EdCxAzCheia) == 0) {  // Se a Caixa Azul precisa encher,
      CxAzPrecEncher = true;             // Sinaliza que a caixa precisa encher,
    }
    if ((digitalRead(BoiaCxAzul) == 0) && digitalRead(EdCxAzCheia) == 1) {  // Se a caixa azul esta cheia,
      CxAzCheia = true;              // sinaliza que a caixa esta cheia para zerar o temporizador de bomba ligada
      CxAzPrecEncher = false;        // Sinaliza que a caixa nao precisa encher,
    }
    if (CxAzNivBaixo == 1) {         // Se o Indicador de Nivel Baixo esta Ativo,
      if (CxAzPrecEncher) {          // e se a Caixa Azul precisa encher,
        EstadoCxAz = 1;              // faz o Estado da Caixa Azul = Precisa Encher Nivel Baixo
      }
      else {                         // Se ha Indicacao de Nivel Baixo e Indicacao de que a caixa nao precisa encher,
        EstadoCxAz = 4;              // faz o Estado da Caixa Azul = Falha de Sinalizacao
      }
    }
    else {                           // Se nao ha Indicacao de Nivel Baixo,
      if (CxAzPrecEncher) {          // e se a Caixa Azul precisa encher,
        EstadoCxAz = 2;              // faz o Estado da Caixa Azul = Precisa Encher Nivel Normal
      }
      else {                         // Se nao ha indicacao de nivel baixo e se a Caixa Azul esta cheia,
        EstadoCxAz = 3;              // faz o Estado da Caixa Azul = Cheia
      }
    }
  }
  else {                             // Se o circuito de sinalizacao da Caixa Azul esta desligado,
    EstadoCxAz = 0;                  // faz o Estado da Caixa Azul = Indefinido
  }
  
} // Fim da Rotina VerifEstCxAz()


//*********************************************************************************************************************
// Nome da Rotina: ControleLEDs()                                                                                     *
//                                                                                                                    *
// Funcao: controla os LEDs da placa de sinalizacao                                                                   *
//                                                                                                                    *
// Entrada: nao tem                                                                                                   *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void ControleLEDs() {

  LedVerde1(ModoOperacao); // Sinaliza Modo de Operação Normal no LED Verde 1
  
  if ((FalhaInversor1 == 1 || FalhaInversor2 == 1)) {
    LedAmarelo(1);  // O LED amarelo sinaliza falha no Inversor 1 ou Inversor 2
  }
  else {
    LedAmarelo(0);
  }

  LedVerde2(EstadoInversor2); // Sinaliza o Inversor 2 ligado e funcionando no LED Verde 2


  if (EstadoRede == 0) { // Sinaliza falta de CA na Rede no LED Verde 3
    LedVerde3(1);
  }
  else {
    LedVerde3(0);
  }

  LedAzul1(Carga1);
  LedAzul2(Carga2);
  LedAzul3(Carga3);
  LedAzul4(ModoControle);
   
} // Fim da Rotina ControleLEDs()


//*********************************************************************************************************************
// Nome da Rotinas: LedVerde1 (primeiro LED de cima), LedAmarelo, LedVerde2 (abaixo do LED amarelo), LedVerde3        *
//                  LedAzul1 (LED azul de cima), LedAzul2, LedAzul3, LedAzul4                                         *
//                                                                                                                    *
// Funcao: Acende ou Apaga os LEDs                                                                                    *
//                                                                                                                    *
// Entrada: byte - (1 = Acende) / (0 = Apaga)                                                                         *
// Saidas: nenhuma                                                                                                    *
//                                                                                                                    *
//*********************************************************************************************************************
//
void LedVerde1 (byte liga) {
  digitalWrite(SD08, liga);
}

void LedAmarelo (byte liga) {
  digitalWrite(SD04, liga);
}

void LedVerde2 (byte liga) {
  digitalWrite(SD05, liga);
}

void LedVerde3 (byte liga) {
  digitalWrite(SD09, liga);
}

void LedAzul1 (byte liga) {
  digitalWrite(SD12, liga);
}

void LedAzul2 (byte liga) {
  digitalWrite(SD13, liga);
}

void LedAzul3 (byte liga) {
  digitalWrite(SD14, liga);
}

void LedAzul4 (byte liga) {
  digitalWrite(SD15, liga);
}


//*********************************************************************************************************************
// Nome da Rotina: Temporizacao (chamada por interrupcao de timer)                                                    *
//                                                                                                                    *
// Funcao: incrementa o contador de tempo (Contador1) se o flag = true (FlagCont1)                                    *
//                                                                                                                    *
// Entradas: nenhuma                                                                                                  *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void Temporizacao() {

  if (HabTmpBombaLig) {
    TmpBombaLig += 1;
  }

  if (HabTmpCxAzNvBx) {
    TmpCxAzNvBx += 1; 
  }
      
} // fim da rotina Temporizacao

//*********************************************************************************************************************
// Nome da Rotina: ByteH                                                                                              *
//                                                                                                                    *
// Funcao: obtem o byte mais significativo de um inteiro de dois bytes                                                *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: byte mais significativo                                                                                     *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte ByteH(double valor) {
  byte BH = valor / 256;
  return(BH);
}

//*********************************************************************************************************************
// Nome da Rotina: ByteL                                                                                              *
//                                                                                                                    *
// Funcao: obtem o byte menos significativo de um inteiro de dois bytes                                               *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: byte menos significativo                                                                                    *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte ByteL(double valor) {
  byte BH = valor / 256;
  byte BL = valor - (256 * BH);
  return(BL);
}

//*********************************************************************************************************************
// Nome da Rotina: ByteHigh                                                                                           *
//                                                                                                                    *
// Funcao: obtem o byte mais significativo de um inteiro de tres bytes                                                *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: byte mais significativo                                                                                     *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte ByteHigh(double valor) {
  byte BH = valor / 65536;
  byte BM = (valor - (65536 * BH)) / 256;
  return(BH);
}

//*********************************************************************************************************************
// Nome da Rotina: ByteMid                                                                                            *
//                                                                                                                    *
// Funcao: obtem o byte do meio de um inteiro de tres bytes                                                           *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: byte mais significativo                                                                                     *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte ByteMid(double valor) {
  byte BH = valor / 65536;
  byte BM = (valor - (65536 * BH)) / 256;
  return(BM);
}

//*********************************************************************************************************************
// Nome da Rotina: ByteLow                                                                                            *
//                                                                                                                    *
// Funcao: obtem o byte menos significativo de um inteiro de tres bytes                                               *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: byte menos significativo                                                                                    *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte ByteLow(double valor) {
  byte BH = valor / 65536;
  double V1 = valor - (65536 * BH);
  byte BM = V1 / 256;
  byte BL = V1 - (256 * BM);
  return(BL);
}

//*********************************************************************************************************************
// Nome da Rotina: Centena                                                                                            *
//                                                                                                                    *
// Funcao: obtem a centena mais significativa de um inteiro na faixa de 0000 a 9999                                   *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: centena mais significativa                                                                                  *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte Centena(unsigned int valor) {
  byte CH = valor / 100;
  return(CH);
}

//*********************************************************************************************************************
// Nome da Rotina: Dezena                                                                                             *
//                                                                                                                    *
// Funcao: obtem a dezena de um inteiro na faixa de 0000 a 9999                                  *
//                                                                                                                    *
// Entrada: valor inteiro                                                                                             *
// Saida: centena menos significativa                                                                                 *
//                                                                                                                    *
//*********************************************************************************************************************
//
byte Dezena(unsigned int valor) {
  byte CH = valor / 100;
  byte CL = valor - (100 * CH);
  return(CL);
}

//*********************************************************************************************************************
// Nome da Rotina: DoisBytesParaInt                                                                                   *
//                                                                                                                    *
// Funcao: converte dois bytes para um inteiro                                                                        *
//                                                                                                                    *
// Entrada: byte menos significativo, byte mais significativo                                                         *
// Saida: valor inteiro                                                                                               *
//                                                                                                                    *
//*********************************************************************************************************************
//
int DoisBytesParaInt(byte BL, byte BH) {
  int ByteL = BL;
  int ByteH = BH;
  if (BL < 0) { ByteL = 256 + BL; }
  if (BH < 0) { ByteH = 256 + BH; }
  return (ByteL + 256*ByteH);
}


//*********************************************************************************************************************
// Nome da Rotina: LigaInversor1() (Inversor 1 é o da esquerda que alimenta a bomba de água do poço)                  *
//                                                                                                                    *
// Funcao: liga o Inversor 1                                                                                          *
//                                                                                                                    *
// Entradas: nenhuma                                                                                                  *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void LigaInversor1() {

  if (FalhaInversor1 == 0) {           // Se o Inversor 1 nao esta em falha,
    if (digitalRead(SD01) == LOW) {    // se o Inversor 1 esta desligado,
        digitalWrite(SD01, HIGH);      // Liga o Inversor 1
    }
  }
}


//*********************************************************************************************************************
// Nome da Rotina: DesligaInversor1() (Inversor 1 é o da esquerda que alimenta a bomba de água do poço)               *
//                                                                                                                    *
// Funcao: desliga o Inversor 1                                                                                       *
//                                                                                                                    *
// Entradas: nenhuma                                                                                                  *
// Saidas: nenhuma                                                                                                    *
//*********************************************************************************************************************
//
void DesligaInversor1() {

  if (digitalRead(SD01) == HIGH) { // Se o Inversor 1 esta ligado e CT4 esta para o Inversor 1
    digitalWrite(SD01, LOW);      // desliga o Inversor 1 e passa a CT4 para a rede,
  }
  
}


//*********************************************************************************************************************
// Nome da Rotina: VerificaInversor1 (Inversor 1 é o da esquerda que alimenta a bomba de água do poço)                *
//                                                                                                                    *
// Funcao: verifica o funcionamento do Inversor 1, liga os indicadores de alarme e aciona os LEDs                     *
//                                                                                                                    *
// Entrada: SD01 => (0 = Inversor Desligado / (1 = Inversor Ligado)                                                   *
// Saida: variaveis de alarme                                                                                         *
//*********************************************************************************************************************
//
void VerificaInversor1() {

  if (digitalRead(SD01) == HIGH) {    // Se o Circuito de Controle do Inversor 1 esta ligado,
    if (digitalRead(DJEINV1) == 1) {  // se o disjuntor de entrada 24Vcc esta ligado,
      
      if (VSInv1 >= VMinInv1) {       // Se a saida 220VCA esta com a tensao normal:
        if (VSInv1 < VMaxInv1) {
          EstadoInversor1 = 1;        // Sinaliza Inversor 1 Ligado e Funcionando
          CTIV1 = 0;                  // Zera o contador de tempo de espera para o inversor ligar
        }
      }
      if (VSInv1 < VMinInv1) {        // Caso a tensao de saida do Inversor 1 esteja baixa,
        CTIV1 = CTIV1 + 1;            // incrementa o contador de espera para o inversor ligar.
        if (CTIV1 > CTRI) {           // Se o inversor nao voltar ao normal em 5 segundos:
          CTIV1 = 0;                  //   1) Zera o contador de espera
          FalhaInversor1 = 1;         //   2) Liga alarme sinalizando ocorrencia de falha no inversor
          SubTensaoInv1 = 1;          //   3) Liga o indicador que indica subtensao na saida do inversor
          EstadoInversor1 = 0;        //   4) Atualiza o estado do inversor 1
        }
      }
      if (VSInv1 >= VMaxInv1) {       // Se a tensao de saida do Inversor 1 esta alta
          FalhaInversor1 = 1;         // Liga alarme sinalizando ocorrencia de falha no inversor
          SobreTensaoInv1 = 1;        // Liga o indicador de sobretensao na saida do inversor
          EstadoInversor1 = 0;        // Atualiza o estado do inversor 1
      }
    
      if (TDInv1 >= LiTDInv) {        // Se a temperatura do driver do Inversor 1 esta alta,
        FalhaInversor1 = 1;           // liga alarme sinalizando ocorrencia de falha no inversor,
        SobreTempDrInv1 = 1;          // liga o indicador sobretemperatura no driver do inversor,
        EstadoInversor1 = 0;          // e atualiza o estado do inversor 1
      }

      if (TTInv1 >= LiTTInv) {        // Se a temperatura do transformador do Inversor 1 esta alta,
        FalhaInversor1 = 1;           // liga alarme sinalizando ocorrencia de falha no inversor,
        SobreTempTrInv1 = 1;          // liga o indicador que indica sobretemperatura no transformador do inversor,
        EstadoInversor1 = 0;          // e atualiza o estado do inversor 1
      }

      if (IEInv1 >= IMaxInv1) {       // Se a corrente de entrada do Inversor 1 esta alta,
        FalhaInversor1 = 1;           // liga alarme sinalizando ocorrencia de falha no inversor,
        SobreCorrenteInv1 = 1;        // liga o indicador que indica sobretemperatura no transformador do inversor,
      }
    }
    else {                            // Se o disjuntor de entrada 24Vcc esta aberto,
      //FalhaInversor1 = 1;           // liga alarme sinalizando ocorrencia de falha no inversor,
      //DisjAbertoIv1 = 1;            // liga indicador identificando a falha
      //EstadoInversor1 = 0;          // Atualiza o estado do inversor 1
    }

    if (FalhaInversor1 == 1) {
      DesligaInversor1();
    }

    if ((EstadoCxAz < 1) || (EstadoCxAz > 2)) {  // Se a Caixa Azul esta cheia ou ha falha,
      DesligaInversor1();                        // desliga o Inversor 1
    }
  }
  else {                                         // Se o Inversor 1 esta desligado,
    EstadoInversor1 = 0;                         // sinaliza
  }

}
