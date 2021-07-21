//
// Created by wirano on 2021/7/21.
//

#include "table.h"

//const double a = 0.03095;
//const double b = 3.532E-06;
//const double c = 8.614E-08;
//const double d = 1.092E-05;

//const double a = 0.004908;
//const double b = 4.862e-06;
//const double c = 2.654e-16;
//const double d = 2.157e-05;

//const double a = 23.04;
//const double b = 0.7214;
//const double c = 1.06;
//const double d = 3.98;

//const double a = 0.01431;
//const double b = 4.213e-06;
//const double c = 1.617e-15;
//const double d = 2.064e-05;


//const double p1 = 9.792e-44;
//const double p2 = -9.97e-37;
//const double p3 = 4.298e-30;
//const double p4 = -1.011e-23;
//const double p5 = 1.38e-17;
//const double p6 = -1.052e-11;
//const double p7 = 3.498e-06;
//const double p8 = 0.3071;
//const double p9 = -3.799e+05;


//const double p1 = 2.443e-26;
//const double p2 = -2.597e-19;
//const double p3 = 1.151e-12;
//const double p4 = -2.719e-06;
//const double p5 = 3.613;
//const double p6 = -2.561e+06;
//const double p7 = 7.564e+11;

//const double p1 =   3.497e-14;
//const double p2 =   -1.82e-07;
//const double p3 =      0.3159;
//const double p4 =  -1.829e+05;

//const double p1 =   4.208e-09;
//const double p2 =    -0.01458;
//const double p3 =   1.267e+04;


//const double a0 = 2.609e+04;
//const double a1 = 3.895e+04;
//const double b1 = 5990;
//const double a2 = 1.543e+04;
//const double b2 = 4880;
//const double a3 = 2529;
//const double b3 = 1253;
//const double w = 1.604e-05;


//const double a1 = 25.21;
//const double b1 = 1.826e+06;
//const double c1 = 1.058e+04;
//const double a2 = 13.57;
//const double b2 = 1.807e+06;
//const double c2 = 1.146e+04;
//const double a3 = 11.73;
//const double b3 = 1.825e+06;
//const double c3 = 4.586e+05;
//const double a4 = 2.796;
//const double b4 = 1.784e+06;
//const double c4 = 6878;
//const double a5 = 2.915;
//const double b5 = 1.772e+06;
//const double c5 = 1788;
//const double a6 = 3.975;
//const double b6 = 1.792e+06;
//const double c6 = 7139;
//const double a7 = 0.9086;
//const double b7 = 1.638e+06;
//const double c7 = 2.259e+04;
//const double a8 = 34.76;
//const double b8 = 1.878e+06;
//const double c8 = 1.693e+05;

//const double a1 = 8.726;
//const double b1 = 1.802e+06;
//const double c1 = 5756;
//const double a2 = 8.232;
//const double b2 = 1.794e+06;
//const double c2 = 1.466e+04;
//const double a3 = 0;
//const double b3 = 1.774e+06;
//const double c3 = 188.4;
//const double a4 = 36.28;
//const double b4 = 1.785e+06;
//const double c4 = 9.23e+04;

//const double a1 = 2.291e+14;
//const double b1 = 2.476e+06;
//const double c1 = 1.201e+05;
//const double a2 = 10.76;
//const double b2 = 1.803e+06;
//const double c2 = 2.768e+04;
//const double a3 = 5.187;
//const double b3 = 1.777e+06;
//const double c3 = 6.861e+04;
//const double a4 = 1.826e+13;
//const double b4 = 1.331e+07;
//const double c4 = 2.211e+06;

//const double a1 = 1.431e+14;
//const double b1 = 2.288e+06;
//const double c1 = 8.523e+04;
//const double a2 = 1.453;
//const double b2 = 1.792e+06;
//const double c2 = 1.302e+04;
//const double a3 = 2941;
//const double b3 = 2.373e+06;
//const double c3 = 2.539e+05;
//const double a4 = 1.173e+05;
//const double b4 = 5.672e+06;
//const double c4 = 1.34e+06;

//const double a1 = 1.136e+15;
//const double b1 = 3.671e+06;
//const double c1 = 3.304e+05;
//const double a2 = 1.837;
//const double b2 = 1.786e+06;
//const double c2 = 1.036e+04;
//const double a3 = 1.33;
//const double b3 = 1.762e+06;
//const double c3 = 1.206e+04;
//const double a4 = 7.864e+06;
//const double b4 = 6.625e+06;
//const double c4 = 1.373e+06;

//const double a1 = 194;
//const double b1 = 1.863e+06;
//const double c1 = 2.868e+04;
//const double a2 = 7.197;
//const double b2 = 1.798e+06;
//const double c2 = 2.484e+04;
//const double a3 = 530.1;
//const double b3 = 2.47e+06;
//const double c3 = 3.876e+05;
//const double a4 = 1.353;
//const double b4 = 1.755e+06;
//const double c4 = 1.279e+04;
//const double a5 = 14.43;
//const double b5 = 2.033e+06;
//const double c5 = 5.751e+05;

//const double a1 = 2.874;
//const double b1 = 1.754e+06;
//const double c1 = 1.488e+04;
//const double a2 = 0.4416;
//const double b2 = 1.719e+06;
//const double c2 = 9437;
//const double a3 = 3.806;
//const double b3 = 1.768e+06;
//const double c3 = 8.892e+04;
//const double a4 = 1.73e+15;
//const double b4 = 1.562e+07;
//const double c4 = 2.454e+06;

//const double a1 = 1.054e+15;
//const double b1 = 3.526e+06;
//const double c1 = 3.053e+05;
//const double a2 = 2.672;
//const double b2 = 1.788e+06;
//const double c2 = 1.047e+04;
//const double a3 = 1.389;
//const double b3 = 1.763e+06;
//const double c3 = 9646;
//const double a4 = 1.199;
//const double b4 = 1.748e+06;
//const double c4 = 9302;
//const double a5 = 2.621e+07;
//const double b5 = 7.178e+06;
//const double c5 = 1.46e+06;

//const double a1 = 4.345;
//const double b1 = 1.743e+06;
//const double c1 = 6.402e+04;
//const double a2 = -0.3796;
//const double b2 = 1.708e+06;
//const double c2 = 8627;
//const double a3 = 1.311;
//const double b3 = 1.737e+06;
//const double c3 = 1.177e+04;
//const double a4 = 1.597e+14;
//const double b4 = 2.193e+06;
//const double c4 = 7.912e+04;
//const double a5 = 180.7;
//const double b5 = 2.831e+06;
//const double c5 = 7.44e+05;

//const double a1 = -2296065785.67983;
//const double a2 = -42.9396607798366;
//const double a3 = 71.1202538996357;
//const double a4 = 1.97005277208379;
//const double a5 = 2.65888051863607;
//const double b1 = -14319109714.5842;
//const double b2 = 1727891.16313180;
//const double b3 = 1779989.61935560;
//const double b4 = 1752249.48007110;
//const double b5 = 1301394.58816997;
//const double c1 = 29086654.4946966;
//const double c2 = 174681.058177461;
//const double c3 = 221054.380920647;
//const double c4 = 11281.2203781537;
//const double c5 = 172009.8260267170;

//分段拟合31-60
//const double sa1 = 29.18;
//const double sb1 = 1.82e+06;
//const double sc1 = 9457;
//const double sa2 = 5.08;
//const double sb2 = 1.807e+06;
//const double sc2 = 5680;
//const double sa3 = 10.44;
//const double sb3 = 1.797e+06;
//const double sc3 = 1.517e+04;
//const double sa4 = -1.373;
//const double sb4 = 1.792e+06;
//const double sc4 = 3977;
//const double sa5 = 36.46;
//const double sb5 = 1.797e+06;
//const double sc5 = 1.013e+05;

//const double sa1 = 39.0997499875822;
//const double sa2 = 4.54969891573061;
//const double sa3 = 17.4789905466391;
//const double sa4 = -1.29353078245377;
//const double sa5 = 33.2706648594671;
//const double sb1 = 1820348.28936334;
//const double sb2 = 1806697.33574431;
//const double sb3 = 1799434.67168167;
//const double sb4 = 1792137.08628014;
//const double sb5 = 1774328.67041329;
//const double sc1 = 10433.5475450764;
//const double sc2 = 5537.01382155798;
//const double sc3 = 18061.7881116977;
//const double sc4 = 3895.38922030636;
//const double sc5 = 66172.0785627557;

//const double sa1 = 754100000000000;
//const double sa2 = 3.82776401425761;
//const double sa3 = 0.802157285611722;
//const double sa4 = 3.64789928451680;
//const double sa5 = 77.2537607944371;
//const double sb1 = 2524108.05235213;
//const double sb2 = 1806174.52041500;
//const double sb3 = 1796465.11090915;
//const double sb4 = 1789052.37223415;
//const double sb5 = 2008777.69867117;
//const double sc1 = 125872.759968345;
//const double sc2 = 9175.28730247511;
//const double sc3 = 1367.61857462206;
//const double sc4 = 9916.34116648308;
//const double sc5 = 263978.2607868750;

//分段1-60
//const double a1 = 783977865241.027;
//const double a2 = -0.61941699913566;
//const double a3 = 0.239424253756763;
//const double a4 = -107.984760307175;
//const double a5 = 136.148129526825;
//const double b1 = 3797233.68171086;
//const double b2 = 1792393.58824605;
//const double b3 = 1745113.15293356;
//const double b4 = 1770300.47193437;
//const double b5 = 1810771.74656375;
//const double c1 = 404327.926197317;
//const double c2 = 6117.02844524817;
//const double c3 = 11257.9336995076;
//const double c4 = 296917.888697193;
//const double c5 = 325434.564116762;

//分段1-65
const double a1 = 5.65929179818966;
const double a2 = 2.00687106950010;
const double a3 = 1830658.12543766;
const double a4 = -0.268011912196634;
const double a5 = 25.6034725397785;
const double b1 = 1821726.41961590;
const double b2 = 1806636.55249609;
const double b3 = 3225201.86033382;
const double b4 = 1762237.13438571;
const double b5 = 2082452.15883903;
const double c1 = 8170.90470065204;
const double c2 = 8988.23878964219;
const double c3 = 428312.148805530;
const double c4 = 8977.38484966381;
const double c5 = 523885.205601444;

//分段55-90
const double sa1 = 10.4892769496347;
const double sa2 = 1.41910958200022;
const double sa3 = 2.36361177767807;
const double sa4 = 3.19025793247625;
const double sa5 = 113.763410489235;
const double sb1 = 1846533.12854260;
const double sb2 = 1837805.01179982;
const double sb3 = 1834780.70201041;
const double sb4 = 1826612.20056687;
const double sb5 = 1926291.29263996;
const double sc1 = 9885.06150226696;
const double sc2 = 1111.41274226961;
const double sc3 = 2616.53887350419;
const double sc4 = 6778.30996249252;
const double sc5 = 136353.804634021;


// 校准
uint32_t freq_orig_1_65[65] = {
        1180987,
        1249741,
        1304514,
        1362014,
        1402742,
        1448322,
        1484318,
        1514407,
        1539484,
        1561890,
        1583233,
        1600445,
        1617203,
        1632766,
        1644369,
        1656841,
        1666625,
        1676171,
        1685236,
        1694574,
        1701850,
        1708211,
        1715314,
        1722343,
        1727005,
        1732513,
        1736879,
        1741529,
        1745903,
        1750025,
        1754479,
        1758307,
        1761960,
        1765337,
        1768563,
        1771146,
        1773862,
        1776834,
        1779755,
        1782132,
        1784542,
        1787400,
        1789553,
        1792116,
        1793989,
        1795457,
        1797613,
        1798536,
        1800126,
        1801867,
        1803697,
        1804901,
        1806125,
        1808080,
        1809532,
        1811167,
        1812493,
        1813829,
        1814792,
        1815500,
        1816337,
        1817870,
        1818578,
        1819673,
        1820874
};

uint32_t freq_orig_55_90[36] = {
        1810221,
        1811189,
        1812757,
        1814428,
        1815691,
        1816851,
        1817579,
        1818586,
        1819808,
        1820715,
        1821963,
        1822513,
        1823178,
        1824623,
        1825500,
        1826216,
        1827946,
        1829492,
        1831195,
        1831736,
        1832504,
        1833317,
        1833773,
        1834743,
        1835622,
        1836690,
        1837270,
        1838542,
        1839902,
        1840626,
        1841282,
        1841791,
        1842451,
        1843532,
        1844184,
        1844884
};
