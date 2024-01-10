/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


/** Row vector of size: 208 */
real_t state[ 208 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
int lRun2;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 12];
state[1] = acadoVariables.x[lRun1 * 12 + 1];
state[2] = acadoVariables.x[lRun1 * 12 + 2];
state[3] = acadoVariables.x[lRun1 * 12 + 3];
state[4] = acadoVariables.x[lRun1 * 12 + 4];
state[5] = acadoVariables.x[lRun1 * 12 + 5];
state[6] = acadoVariables.x[lRun1 * 12 + 6];
state[7] = acadoVariables.x[lRun1 * 12 + 7];
state[8] = acadoVariables.x[lRun1 * 12 + 8];
state[9] = acadoVariables.x[lRun1 * 12 + 9];
state[10] = acadoVariables.x[lRun1 * 12 + 10];
state[11] = acadoVariables.x[lRun1 * 12 + 11];

state[204] = acadoVariables.u[lRun1 * 4];
state[205] = acadoVariables.u[lRun1 * 4 + 1];
state[206] = acadoVariables.u[lRun1 * 4 + 2];
state[207] = acadoVariables.u[lRun1 * 4 + 3];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 12] = state[0] - acadoVariables.x[lRun1 * 12 + 12];
acadoWorkspace.d[lRun1 * 12 + 1] = state[1] - acadoVariables.x[lRun1 * 12 + 13];
acadoWorkspace.d[lRun1 * 12 + 2] = state[2] - acadoVariables.x[lRun1 * 12 + 14];
acadoWorkspace.d[lRun1 * 12 + 3] = state[3] - acadoVariables.x[lRun1 * 12 + 15];
acadoWorkspace.d[lRun1 * 12 + 4] = state[4] - acadoVariables.x[lRun1 * 12 + 16];
acadoWorkspace.d[lRun1 * 12 + 5] = state[5] - acadoVariables.x[lRun1 * 12 + 17];
acadoWorkspace.d[lRun1 * 12 + 6] = state[6] - acadoVariables.x[lRun1 * 12 + 18];
acadoWorkspace.d[lRun1 * 12 + 7] = state[7] - acadoVariables.x[lRun1 * 12 + 19];
acadoWorkspace.d[lRun1 * 12 + 8] = state[8] - acadoVariables.x[lRun1 * 12 + 20];
acadoWorkspace.d[lRun1 * 12 + 9] = state[9] - acadoVariables.x[lRun1 * 12 + 21];
acadoWorkspace.d[lRun1 * 12 + 10] = state[10] - acadoVariables.x[lRun1 * 12 + 22];
acadoWorkspace.d[lRun1 * 12 + 11] = state[11] - acadoVariables.x[lRun1 * 12 + 23];

for (lRun2 = 0; lRun2 < 144; ++lRun2)
acadoWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 144))] = state[lRun2 + 12];


acadoWorkspace.evGu[lRun1 * 48] = state[156];
acadoWorkspace.evGu[lRun1 * 48 + 1] = state[157];
acadoWorkspace.evGu[lRun1 * 48 + 2] = state[158];
acadoWorkspace.evGu[lRun1 * 48 + 3] = state[159];
acadoWorkspace.evGu[lRun1 * 48 + 4] = state[160];
acadoWorkspace.evGu[lRun1 * 48 + 5] = state[161];
acadoWorkspace.evGu[lRun1 * 48 + 6] = state[162];
acadoWorkspace.evGu[lRun1 * 48 + 7] = state[163];
acadoWorkspace.evGu[lRun1 * 48 + 8] = state[164];
acadoWorkspace.evGu[lRun1 * 48 + 9] = state[165];
acadoWorkspace.evGu[lRun1 * 48 + 10] = state[166];
acadoWorkspace.evGu[lRun1 * 48 + 11] = state[167];
acadoWorkspace.evGu[lRun1 * 48 + 12] = state[168];
acadoWorkspace.evGu[lRun1 * 48 + 13] = state[169];
acadoWorkspace.evGu[lRun1 * 48 + 14] = state[170];
acadoWorkspace.evGu[lRun1 * 48 + 15] = state[171];
acadoWorkspace.evGu[lRun1 * 48 + 16] = state[172];
acadoWorkspace.evGu[lRun1 * 48 + 17] = state[173];
acadoWorkspace.evGu[lRun1 * 48 + 18] = state[174];
acadoWorkspace.evGu[lRun1 * 48 + 19] = state[175];
acadoWorkspace.evGu[lRun1 * 48 + 20] = state[176];
acadoWorkspace.evGu[lRun1 * 48 + 21] = state[177];
acadoWorkspace.evGu[lRun1 * 48 + 22] = state[178];
acadoWorkspace.evGu[lRun1 * 48 + 23] = state[179];
acadoWorkspace.evGu[lRun1 * 48 + 24] = state[180];
acadoWorkspace.evGu[lRun1 * 48 + 25] = state[181];
acadoWorkspace.evGu[lRun1 * 48 + 26] = state[182];
acadoWorkspace.evGu[lRun1 * 48 + 27] = state[183];
acadoWorkspace.evGu[lRun1 * 48 + 28] = state[184];
acadoWorkspace.evGu[lRun1 * 48 + 29] = state[185];
acadoWorkspace.evGu[lRun1 * 48 + 30] = state[186];
acadoWorkspace.evGu[lRun1 * 48 + 31] = state[187];
acadoWorkspace.evGu[lRun1 * 48 + 32] = state[188];
acadoWorkspace.evGu[lRun1 * 48 + 33] = state[189];
acadoWorkspace.evGu[lRun1 * 48 + 34] = state[190];
acadoWorkspace.evGu[lRun1 * 48 + 35] = state[191];
acadoWorkspace.evGu[lRun1 * 48 + 36] = state[192];
acadoWorkspace.evGu[lRun1 * 48 + 37] = state[193];
acadoWorkspace.evGu[lRun1 * 48 + 38] = state[194];
acadoWorkspace.evGu[lRun1 * 48 + 39] = state[195];
acadoWorkspace.evGu[lRun1 * 48 + 40] = state[196];
acadoWorkspace.evGu[lRun1 * 48 + 41] = state[197];
acadoWorkspace.evGu[lRun1 * 48 + 42] = state[198];
acadoWorkspace.evGu[lRun1 * 48 + 43] = state[199];
acadoWorkspace.evGu[lRun1 * 48 + 44] = state[200];
acadoWorkspace.evGu[lRun1 * 48 + 45] = state[201];
acadoWorkspace.evGu[lRun1 * 48 + 46] = state[202];
acadoWorkspace.evGu[lRun1 * 48 + 47] = state[203];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
out[10] = xd[10];
out[11] = xd[11];
out[12] = u[0];
out[13] = u[1];
out[14] = u[2];
out[15] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
out[10] = xd[10];
out[11] = xd[11];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ2[54] = +tmpObjS[54];
tmpQ2[55] = +tmpObjS[55];
tmpQ2[56] = +tmpObjS[56];
tmpQ2[57] = +tmpObjS[57];
tmpQ2[58] = +tmpObjS[58];
tmpQ2[59] = +tmpObjS[59];
tmpQ2[60] = +tmpObjS[60];
tmpQ2[61] = +tmpObjS[61];
tmpQ2[62] = +tmpObjS[62];
tmpQ2[63] = +tmpObjS[63];
tmpQ2[64] = +tmpObjS[64];
tmpQ2[65] = +tmpObjS[65];
tmpQ2[66] = +tmpObjS[66];
tmpQ2[67] = +tmpObjS[67];
tmpQ2[68] = +tmpObjS[68];
tmpQ2[69] = +tmpObjS[69];
tmpQ2[70] = +tmpObjS[70];
tmpQ2[71] = +tmpObjS[71];
tmpQ2[72] = +tmpObjS[72];
tmpQ2[73] = +tmpObjS[73];
tmpQ2[74] = +tmpObjS[74];
tmpQ2[75] = +tmpObjS[75];
tmpQ2[76] = +tmpObjS[76];
tmpQ2[77] = +tmpObjS[77];
tmpQ2[78] = +tmpObjS[78];
tmpQ2[79] = +tmpObjS[79];
tmpQ2[80] = +tmpObjS[80];
tmpQ2[81] = +tmpObjS[81];
tmpQ2[82] = +tmpObjS[82];
tmpQ2[83] = +tmpObjS[83];
tmpQ2[84] = +tmpObjS[84];
tmpQ2[85] = +tmpObjS[85];
tmpQ2[86] = +tmpObjS[86];
tmpQ2[87] = +tmpObjS[87];
tmpQ2[88] = +tmpObjS[88];
tmpQ2[89] = +tmpObjS[89];
tmpQ2[90] = +tmpObjS[90];
tmpQ2[91] = +tmpObjS[91];
tmpQ2[92] = +tmpObjS[92];
tmpQ2[93] = +tmpObjS[93];
tmpQ2[94] = +tmpObjS[94];
tmpQ2[95] = +tmpObjS[95];
tmpQ2[96] = +tmpObjS[96];
tmpQ2[97] = +tmpObjS[97];
tmpQ2[98] = +tmpObjS[98];
tmpQ2[99] = +tmpObjS[99];
tmpQ2[100] = +tmpObjS[100];
tmpQ2[101] = +tmpObjS[101];
tmpQ2[102] = +tmpObjS[102];
tmpQ2[103] = +tmpObjS[103];
tmpQ2[104] = +tmpObjS[104];
tmpQ2[105] = +tmpObjS[105];
tmpQ2[106] = +tmpObjS[106];
tmpQ2[107] = +tmpObjS[107];
tmpQ2[108] = +tmpObjS[108];
tmpQ2[109] = +tmpObjS[109];
tmpQ2[110] = +tmpObjS[110];
tmpQ2[111] = +tmpObjS[111];
tmpQ2[112] = +tmpObjS[112];
tmpQ2[113] = +tmpObjS[113];
tmpQ2[114] = +tmpObjS[114];
tmpQ2[115] = +tmpObjS[115];
tmpQ2[116] = +tmpObjS[116];
tmpQ2[117] = +tmpObjS[117];
tmpQ2[118] = +tmpObjS[118];
tmpQ2[119] = +tmpObjS[119];
tmpQ2[120] = +tmpObjS[120];
tmpQ2[121] = +tmpObjS[121];
tmpQ2[122] = +tmpObjS[122];
tmpQ2[123] = +tmpObjS[123];
tmpQ2[124] = +tmpObjS[124];
tmpQ2[125] = +tmpObjS[125];
tmpQ2[126] = +tmpObjS[126];
tmpQ2[127] = +tmpObjS[127];
tmpQ2[128] = +tmpObjS[128];
tmpQ2[129] = +tmpObjS[129];
tmpQ2[130] = +tmpObjS[130];
tmpQ2[131] = +tmpObjS[131];
tmpQ2[132] = +tmpObjS[132];
tmpQ2[133] = +tmpObjS[133];
tmpQ2[134] = +tmpObjS[134];
tmpQ2[135] = +tmpObjS[135];
tmpQ2[136] = +tmpObjS[136];
tmpQ2[137] = +tmpObjS[137];
tmpQ2[138] = +tmpObjS[138];
tmpQ2[139] = +tmpObjS[139];
tmpQ2[140] = +tmpObjS[140];
tmpQ2[141] = +tmpObjS[141];
tmpQ2[142] = +tmpObjS[142];
tmpQ2[143] = +tmpObjS[143];
tmpQ2[144] = +tmpObjS[144];
tmpQ2[145] = +tmpObjS[145];
tmpQ2[146] = +tmpObjS[146];
tmpQ2[147] = +tmpObjS[147];
tmpQ2[148] = +tmpObjS[148];
tmpQ2[149] = +tmpObjS[149];
tmpQ2[150] = +tmpObjS[150];
tmpQ2[151] = +tmpObjS[151];
tmpQ2[152] = +tmpObjS[152];
tmpQ2[153] = +tmpObjS[153];
tmpQ2[154] = +tmpObjS[154];
tmpQ2[155] = +tmpObjS[155];
tmpQ2[156] = +tmpObjS[156];
tmpQ2[157] = +tmpObjS[157];
tmpQ2[158] = +tmpObjS[158];
tmpQ2[159] = +tmpObjS[159];
tmpQ2[160] = +tmpObjS[160];
tmpQ2[161] = +tmpObjS[161];
tmpQ2[162] = +tmpObjS[162];
tmpQ2[163] = +tmpObjS[163];
tmpQ2[164] = +tmpObjS[164];
tmpQ2[165] = +tmpObjS[165];
tmpQ2[166] = +tmpObjS[166];
tmpQ2[167] = +tmpObjS[167];
tmpQ2[168] = +tmpObjS[168];
tmpQ2[169] = +tmpObjS[169];
tmpQ2[170] = +tmpObjS[170];
tmpQ2[171] = +tmpObjS[171];
tmpQ2[172] = +tmpObjS[172];
tmpQ2[173] = +tmpObjS[173];
tmpQ2[174] = +tmpObjS[174];
tmpQ2[175] = +tmpObjS[175];
tmpQ2[176] = +tmpObjS[176];
tmpQ2[177] = +tmpObjS[177];
tmpQ2[178] = +tmpObjS[178];
tmpQ2[179] = +tmpObjS[179];
tmpQ2[180] = +tmpObjS[180];
tmpQ2[181] = +tmpObjS[181];
tmpQ2[182] = +tmpObjS[182];
tmpQ2[183] = +tmpObjS[183];
tmpQ2[184] = +tmpObjS[184];
tmpQ2[185] = +tmpObjS[185];
tmpQ2[186] = +tmpObjS[186];
tmpQ2[187] = +tmpObjS[187];
tmpQ2[188] = +tmpObjS[188];
tmpQ2[189] = +tmpObjS[189];
tmpQ2[190] = +tmpObjS[190];
tmpQ2[191] = +tmpObjS[191];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[8];
tmpQ1[9] = + tmpQ2[9];
tmpQ1[10] = + tmpQ2[10];
tmpQ1[11] = + tmpQ2[11];
tmpQ1[12] = + tmpQ2[16];
tmpQ1[13] = + tmpQ2[17];
tmpQ1[14] = + tmpQ2[18];
tmpQ1[15] = + tmpQ2[19];
tmpQ1[16] = + tmpQ2[20];
tmpQ1[17] = + tmpQ2[21];
tmpQ1[18] = + tmpQ2[22];
tmpQ1[19] = + tmpQ2[23];
tmpQ1[20] = + tmpQ2[24];
tmpQ1[21] = + tmpQ2[25];
tmpQ1[22] = + tmpQ2[26];
tmpQ1[23] = + tmpQ2[27];
tmpQ1[24] = + tmpQ2[32];
tmpQ1[25] = + tmpQ2[33];
tmpQ1[26] = + tmpQ2[34];
tmpQ1[27] = + tmpQ2[35];
tmpQ1[28] = + tmpQ2[36];
tmpQ1[29] = + tmpQ2[37];
tmpQ1[30] = + tmpQ2[38];
tmpQ1[31] = + tmpQ2[39];
tmpQ1[32] = + tmpQ2[40];
tmpQ1[33] = + tmpQ2[41];
tmpQ1[34] = + tmpQ2[42];
tmpQ1[35] = + tmpQ2[43];
tmpQ1[36] = + tmpQ2[48];
tmpQ1[37] = + tmpQ2[49];
tmpQ1[38] = + tmpQ2[50];
tmpQ1[39] = + tmpQ2[51];
tmpQ1[40] = + tmpQ2[52];
tmpQ1[41] = + tmpQ2[53];
tmpQ1[42] = + tmpQ2[54];
tmpQ1[43] = + tmpQ2[55];
tmpQ1[44] = + tmpQ2[56];
tmpQ1[45] = + tmpQ2[57];
tmpQ1[46] = + tmpQ2[58];
tmpQ1[47] = + tmpQ2[59];
tmpQ1[48] = + tmpQ2[64];
tmpQ1[49] = + tmpQ2[65];
tmpQ1[50] = + tmpQ2[66];
tmpQ1[51] = + tmpQ2[67];
tmpQ1[52] = + tmpQ2[68];
tmpQ1[53] = + tmpQ2[69];
tmpQ1[54] = + tmpQ2[70];
tmpQ1[55] = + tmpQ2[71];
tmpQ1[56] = + tmpQ2[72];
tmpQ1[57] = + tmpQ2[73];
tmpQ1[58] = + tmpQ2[74];
tmpQ1[59] = + tmpQ2[75];
tmpQ1[60] = + tmpQ2[80];
tmpQ1[61] = + tmpQ2[81];
tmpQ1[62] = + tmpQ2[82];
tmpQ1[63] = + tmpQ2[83];
tmpQ1[64] = + tmpQ2[84];
tmpQ1[65] = + tmpQ2[85];
tmpQ1[66] = + tmpQ2[86];
tmpQ1[67] = + tmpQ2[87];
tmpQ1[68] = + tmpQ2[88];
tmpQ1[69] = + tmpQ2[89];
tmpQ1[70] = + tmpQ2[90];
tmpQ1[71] = + tmpQ2[91];
tmpQ1[72] = + tmpQ2[96];
tmpQ1[73] = + tmpQ2[97];
tmpQ1[74] = + tmpQ2[98];
tmpQ1[75] = + tmpQ2[99];
tmpQ1[76] = + tmpQ2[100];
tmpQ1[77] = + tmpQ2[101];
tmpQ1[78] = + tmpQ2[102];
tmpQ1[79] = + tmpQ2[103];
tmpQ1[80] = + tmpQ2[104];
tmpQ1[81] = + tmpQ2[105];
tmpQ1[82] = + tmpQ2[106];
tmpQ1[83] = + tmpQ2[107];
tmpQ1[84] = + tmpQ2[112];
tmpQ1[85] = + tmpQ2[113];
tmpQ1[86] = + tmpQ2[114];
tmpQ1[87] = + tmpQ2[115];
tmpQ1[88] = + tmpQ2[116];
tmpQ1[89] = + tmpQ2[117];
tmpQ1[90] = + tmpQ2[118];
tmpQ1[91] = + tmpQ2[119];
tmpQ1[92] = + tmpQ2[120];
tmpQ1[93] = + tmpQ2[121];
tmpQ1[94] = + tmpQ2[122];
tmpQ1[95] = + tmpQ2[123];
tmpQ1[96] = + tmpQ2[128];
tmpQ1[97] = + tmpQ2[129];
tmpQ1[98] = + tmpQ2[130];
tmpQ1[99] = + tmpQ2[131];
tmpQ1[100] = + tmpQ2[132];
tmpQ1[101] = + tmpQ2[133];
tmpQ1[102] = + tmpQ2[134];
tmpQ1[103] = + tmpQ2[135];
tmpQ1[104] = + tmpQ2[136];
tmpQ1[105] = + tmpQ2[137];
tmpQ1[106] = + tmpQ2[138];
tmpQ1[107] = + tmpQ2[139];
tmpQ1[108] = + tmpQ2[144];
tmpQ1[109] = + tmpQ2[145];
tmpQ1[110] = + tmpQ2[146];
tmpQ1[111] = + tmpQ2[147];
tmpQ1[112] = + tmpQ2[148];
tmpQ1[113] = + tmpQ2[149];
tmpQ1[114] = + tmpQ2[150];
tmpQ1[115] = + tmpQ2[151];
tmpQ1[116] = + tmpQ2[152];
tmpQ1[117] = + tmpQ2[153];
tmpQ1[118] = + tmpQ2[154];
tmpQ1[119] = + tmpQ2[155];
tmpQ1[120] = + tmpQ2[160];
tmpQ1[121] = + tmpQ2[161];
tmpQ1[122] = + tmpQ2[162];
tmpQ1[123] = + tmpQ2[163];
tmpQ1[124] = + tmpQ2[164];
tmpQ1[125] = + tmpQ2[165];
tmpQ1[126] = + tmpQ2[166];
tmpQ1[127] = + tmpQ2[167];
tmpQ1[128] = + tmpQ2[168];
tmpQ1[129] = + tmpQ2[169];
tmpQ1[130] = + tmpQ2[170];
tmpQ1[131] = + tmpQ2[171];
tmpQ1[132] = + tmpQ2[176];
tmpQ1[133] = + tmpQ2[177];
tmpQ1[134] = + tmpQ2[178];
tmpQ1[135] = + tmpQ2[179];
tmpQ1[136] = + tmpQ2[180];
tmpQ1[137] = + tmpQ2[181];
tmpQ1[138] = + tmpQ2[182];
tmpQ1[139] = + tmpQ2[183];
tmpQ1[140] = + tmpQ2[184];
tmpQ1[141] = + tmpQ2[185];
tmpQ1[142] = + tmpQ2[186];
tmpQ1[143] = + tmpQ2[187];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[192];
tmpR2[1] = +tmpObjS[193];
tmpR2[2] = +tmpObjS[194];
tmpR2[3] = +tmpObjS[195];
tmpR2[4] = +tmpObjS[196];
tmpR2[5] = +tmpObjS[197];
tmpR2[6] = +tmpObjS[198];
tmpR2[7] = +tmpObjS[199];
tmpR2[8] = +tmpObjS[200];
tmpR2[9] = +tmpObjS[201];
tmpR2[10] = +tmpObjS[202];
tmpR2[11] = +tmpObjS[203];
tmpR2[12] = +tmpObjS[204];
tmpR2[13] = +tmpObjS[205];
tmpR2[14] = +tmpObjS[206];
tmpR2[15] = +tmpObjS[207];
tmpR2[16] = +tmpObjS[208];
tmpR2[17] = +tmpObjS[209];
tmpR2[18] = +tmpObjS[210];
tmpR2[19] = +tmpObjS[211];
tmpR2[20] = +tmpObjS[212];
tmpR2[21] = +tmpObjS[213];
tmpR2[22] = +tmpObjS[214];
tmpR2[23] = +tmpObjS[215];
tmpR2[24] = +tmpObjS[216];
tmpR2[25] = +tmpObjS[217];
tmpR2[26] = +tmpObjS[218];
tmpR2[27] = +tmpObjS[219];
tmpR2[28] = +tmpObjS[220];
tmpR2[29] = +tmpObjS[221];
tmpR2[30] = +tmpObjS[222];
tmpR2[31] = +tmpObjS[223];
tmpR2[32] = +tmpObjS[224];
tmpR2[33] = +tmpObjS[225];
tmpR2[34] = +tmpObjS[226];
tmpR2[35] = +tmpObjS[227];
tmpR2[36] = +tmpObjS[228];
tmpR2[37] = +tmpObjS[229];
tmpR2[38] = +tmpObjS[230];
tmpR2[39] = +tmpObjS[231];
tmpR2[40] = +tmpObjS[232];
tmpR2[41] = +tmpObjS[233];
tmpR2[42] = +tmpObjS[234];
tmpR2[43] = +tmpObjS[235];
tmpR2[44] = +tmpObjS[236];
tmpR2[45] = +tmpObjS[237];
tmpR2[46] = +tmpObjS[238];
tmpR2[47] = +tmpObjS[239];
tmpR2[48] = +tmpObjS[240];
tmpR2[49] = +tmpObjS[241];
tmpR2[50] = +tmpObjS[242];
tmpR2[51] = +tmpObjS[243];
tmpR2[52] = +tmpObjS[244];
tmpR2[53] = +tmpObjS[245];
tmpR2[54] = +tmpObjS[246];
tmpR2[55] = +tmpObjS[247];
tmpR2[56] = +tmpObjS[248];
tmpR2[57] = +tmpObjS[249];
tmpR2[58] = +tmpObjS[250];
tmpR2[59] = +tmpObjS[251];
tmpR2[60] = +tmpObjS[252];
tmpR2[61] = +tmpObjS[253];
tmpR2[62] = +tmpObjS[254];
tmpR2[63] = +tmpObjS[255];
tmpR1[0] = + tmpR2[12];
tmpR1[1] = + tmpR2[13];
tmpR1[2] = + tmpR2[14];
tmpR1[3] = + tmpR2[15];
tmpR1[4] = + tmpR2[28];
tmpR1[5] = + tmpR2[29];
tmpR1[6] = + tmpR2[30];
tmpR1[7] = + tmpR2[31];
tmpR1[8] = + tmpR2[44];
tmpR1[9] = + tmpR2[45];
tmpR1[10] = + tmpR2[46];
tmpR1[11] = + tmpR2[47];
tmpR1[12] = + tmpR2[60];
tmpR1[13] = + tmpR2[61];
tmpR1[14] = + tmpR2[62];
tmpR1[15] = + tmpR2[63];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = +tmpObjSEndTerm[36];
tmpQN2[37] = +tmpObjSEndTerm[37];
tmpQN2[38] = +tmpObjSEndTerm[38];
tmpQN2[39] = +tmpObjSEndTerm[39];
tmpQN2[40] = +tmpObjSEndTerm[40];
tmpQN2[41] = +tmpObjSEndTerm[41];
tmpQN2[42] = +tmpObjSEndTerm[42];
tmpQN2[43] = +tmpObjSEndTerm[43];
tmpQN2[44] = +tmpObjSEndTerm[44];
tmpQN2[45] = +tmpObjSEndTerm[45];
tmpQN2[46] = +tmpObjSEndTerm[46];
tmpQN2[47] = +tmpObjSEndTerm[47];
tmpQN2[48] = +tmpObjSEndTerm[48];
tmpQN2[49] = +tmpObjSEndTerm[49];
tmpQN2[50] = +tmpObjSEndTerm[50];
tmpQN2[51] = +tmpObjSEndTerm[51];
tmpQN2[52] = +tmpObjSEndTerm[52];
tmpQN2[53] = +tmpObjSEndTerm[53];
tmpQN2[54] = +tmpObjSEndTerm[54];
tmpQN2[55] = +tmpObjSEndTerm[55];
tmpQN2[56] = +tmpObjSEndTerm[56];
tmpQN2[57] = +tmpObjSEndTerm[57];
tmpQN2[58] = +tmpObjSEndTerm[58];
tmpQN2[59] = +tmpObjSEndTerm[59];
tmpQN2[60] = +tmpObjSEndTerm[60];
tmpQN2[61] = +tmpObjSEndTerm[61];
tmpQN2[62] = +tmpObjSEndTerm[62];
tmpQN2[63] = +tmpObjSEndTerm[63];
tmpQN2[64] = +tmpObjSEndTerm[64];
tmpQN2[65] = +tmpObjSEndTerm[65];
tmpQN2[66] = +tmpObjSEndTerm[66];
tmpQN2[67] = +tmpObjSEndTerm[67];
tmpQN2[68] = +tmpObjSEndTerm[68];
tmpQN2[69] = +tmpObjSEndTerm[69];
tmpQN2[70] = +tmpObjSEndTerm[70];
tmpQN2[71] = +tmpObjSEndTerm[71];
tmpQN2[72] = +tmpObjSEndTerm[72];
tmpQN2[73] = +tmpObjSEndTerm[73];
tmpQN2[74] = +tmpObjSEndTerm[74];
tmpQN2[75] = +tmpObjSEndTerm[75];
tmpQN2[76] = +tmpObjSEndTerm[76];
tmpQN2[77] = +tmpObjSEndTerm[77];
tmpQN2[78] = +tmpObjSEndTerm[78];
tmpQN2[79] = +tmpObjSEndTerm[79];
tmpQN2[80] = +tmpObjSEndTerm[80];
tmpQN2[81] = +tmpObjSEndTerm[81];
tmpQN2[82] = +tmpObjSEndTerm[82];
tmpQN2[83] = +tmpObjSEndTerm[83];
tmpQN2[84] = +tmpObjSEndTerm[84];
tmpQN2[85] = +tmpObjSEndTerm[85];
tmpQN2[86] = +tmpObjSEndTerm[86];
tmpQN2[87] = +tmpObjSEndTerm[87];
tmpQN2[88] = +tmpObjSEndTerm[88];
tmpQN2[89] = +tmpObjSEndTerm[89];
tmpQN2[90] = +tmpObjSEndTerm[90];
tmpQN2[91] = +tmpObjSEndTerm[91];
tmpQN2[92] = +tmpObjSEndTerm[92];
tmpQN2[93] = +tmpObjSEndTerm[93];
tmpQN2[94] = +tmpObjSEndTerm[94];
tmpQN2[95] = +tmpObjSEndTerm[95];
tmpQN2[96] = +tmpObjSEndTerm[96];
tmpQN2[97] = +tmpObjSEndTerm[97];
tmpQN2[98] = +tmpObjSEndTerm[98];
tmpQN2[99] = +tmpObjSEndTerm[99];
tmpQN2[100] = +tmpObjSEndTerm[100];
tmpQN2[101] = +tmpObjSEndTerm[101];
tmpQN2[102] = +tmpObjSEndTerm[102];
tmpQN2[103] = +tmpObjSEndTerm[103];
tmpQN2[104] = +tmpObjSEndTerm[104];
tmpQN2[105] = +tmpObjSEndTerm[105];
tmpQN2[106] = +tmpObjSEndTerm[106];
tmpQN2[107] = +tmpObjSEndTerm[107];
tmpQN2[108] = +tmpObjSEndTerm[108];
tmpQN2[109] = +tmpObjSEndTerm[109];
tmpQN2[110] = +tmpObjSEndTerm[110];
tmpQN2[111] = +tmpObjSEndTerm[111];
tmpQN2[112] = +tmpObjSEndTerm[112];
tmpQN2[113] = +tmpObjSEndTerm[113];
tmpQN2[114] = +tmpObjSEndTerm[114];
tmpQN2[115] = +tmpObjSEndTerm[115];
tmpQN2[116] = +tmpObjSEndTerm[116];
tmpQN2[117] = +tmpObjSEndTerm[117];
tmpQN2[118] = +tmpObjSEndTerm[118];
tmpQN2[119] = +tmpObjSEndTerm[119];
tmpQN2[120] = +tmpObjSEndTerm[120];
tmpQN2[121] = +tmpObjSEndTerm[121];
tmpQN2[122] = +tmpObjSEndTerm[122];
tmpQN2[123] = +tmpObjSEndTerm[123];
tmpQN2[124] = +tmpObjSEndTerm[124];
tmpQN2[125] = +tmpObjSEndTerm[125];
tmpQN2[126] = +tmpObjSEndTerm[126];
tmpQN2[127] = +tmpObjSEndTerm[127];
tmpQN2[128] = +tmpObjSEndTerm[128];
tmpQN2[129] = +tmpObjSEndTerm[129];
tmpQN2[130] = +tmpObjSEndTerm[130];
tmpQN2[131] = +tmpObjSEndTerm[131];
tmpQN2[132] = +tmpObjSEndTerm[132];
tmpQN2[133] = +tmpObjSEndTerm[133];
tmpQN2[134] = +tmpObjSEndTerm[134];
tmpQN2[135] = +tmpObjSEndTerm[135];
tmpQN2[136] = +tmpObjSEndTerm[136];
tmpQN2[137] = +tmpObjSEndTerm[137];
tmpQN2[138] = +tmpObjSEndTerm[138];
tmpQN2[139] = +tmpObjSEndTerm[139];
tmpQN2[140] = +tmpObjSEndTerm[140];
tmpQN2[141] = +tmpObjSEndTerm[141];
tmpQN2[142] = +tmpObjSEndTerm[142];
tmpQN2[143] = +tmpObjSEndTerm[143];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
tmpQN1[36] = + tmpQN2[36];
tmpQN1[37] = + tmpQN2[37];
tmpQN1[38] = + tmpQN2[38];
tmpQN1[39] = + tmpQN2[39];
tmpQN1[40] = + tmpQN2[40];
tmpQN1[41] = + tmpQN2[41];
tmpQN1[42] = + tmpQN2[42];
tmpQN1[43] = + tmpQN2[43];
tmpQN1[44] = + tmpQN2[44];
tmpQN1[45] = + tmpQN2[45];
tmpQN1[46] = + tmpQN2[46];
tmpQN1[47] = + tmpQN2[47];
tmpQN1[48] = + tmpQN2[48];
tmpQN1[49] = + tmpQN2[49];
tmpQN1[50] = + tmpQN2[50];
tmpQN1[51] = + tmpQN2[51];
tmpQN1[52] = + tmpQN2[52];
tmpQN1[53] = + tmpQN2[53];
tmpQN1[54] = + tmpQN2[54];
tmpQN1[55] = + tmpQN2[55];
tmpQN1[56] = + tmpQN2[56];
tmpQN1[57] = + tmpQN2[57];
tmpQN1[58] = + tmpQN2[58];
tmpQN1[59] = + tmpQN2[59];
tmpQN1[60] = + tmpQN2[60];
tmpQN1[61] = + tmpQN2[61];
tmpQN1[62] = + tmpQN2[62];
tmpQN1[63] = + tmpQN2[63];
tmpQN1[64] = + tmpQN2[64];
tmpQN1[65] = + tmpQN2[65];
tmpQN1[66] = + tmpQN2[66];
tmpQN1[67] = + tmpQN2[67];
tmpQN1[68] = + tmpQN2[68];
tmpQN1[69] = + tmpQN2[69];
tmpQN1[70] = + tmpQN2[70];
tmpQN1[71] = + tmpQN2[71];
tmpQN1[72] = + tmpQN2[72];
tmpQN1[73] = + tmpQN2[73];
tmpQN1[74] = + tmpQN2[74];
tmpQN1[75] = + tmpQN2[75];
tmpQN1[76] = + tmpQN2[76];
tmpQN1[77] = + tmpQN2[77];
tmpQN1[78] = + tmpQN2[78];
tmpQN1[79] = + tmpQN2[79];
tmpQN1[80] = + tmpQN2[80];
tmpQN1[81] = + tmpQN2[81];
tmpQN1[82] = + tmpQN2[82];
tmpQN1[83] = + tmpQN2[83];
tmpQN1[84] = + tmpQN2[84];
tmpQN1[85] = + tmpQN2[85];
tmpQN1[86] = + tmpQN2[86];
tmpQN1[87] = + tmpQN2[87];
tmpQN1[88] = + tmpQN2[88];
tmpQN1[89] = + tmpQN2[89];
tmpQN1[90] = + tmpQN2[90];
tmpQN1[91] = + tmpQN2[91];
tmpQN1[92] = + tmpQN2[92];
tmpQN1[93] = + tmpQN2[93];
tmpQN1[94] = + tmpQN2[94];
tmpQN1[95] = + tmpQN2[95];
tmpQN1[96] = + tmpQN2[96];
tmpQN1[97] = + tmpQN2[97];
tmpQN1[98] = + tmpQN2[98];
tmpQN1[99] = + tmpQN2[99];
tmpQN1[100] = + tmpQN2[100];
tmpQN1[101] = + tmpQN2[101];
tmpQN1[102] = + tmpQN2[102];
tmpQN1[103] = + tmpQN2[103];
tmpQN1[104] = + tmpQN2[104];
tmpQN1[105] = + tmpQN2[105];
tmpQN1[106] = + tmpQN2[106];
tmpQN1[107] = + tmpQN2[107];
tmpQN1[108] = + tmpQN2[108];
tmpQN1[109] = + tmpQN2[109];
tmpQN1[110] = + tmpQN2[110];
tmpQN1[111] = + tmpQN2[111];
tmpQN1[112] = + tmpQN2[112];
tmpQN1[113] = + tmpQN2[113];
tmpQN1[114] = + tmpQN2[114];
tmpQN1[115] = + tmpQN2[115];
tmpQN1[116] = + tmpQN2[116];
tmpQN1[117] = + tmpQN2[117];
tmpQN1[118] = + tmpQN2[118];
tmpQN1[119] = + tmpQN2[119];
tmpQN1[120] = + tmpQN2[120];
tmpQN1[121] = + tmpQN2[121];
tmpQN1[122] = + tmpQN2[122];
tmpQN1[123] = + tmpQN2[123];
tmpQN1[124] = + tmpQN2[124];
tmpQN1[125] = + tmpQN2[125];
tmpQN1[126] = + tmpQN2[126];
tmpQN1[127] = + tmpQN2[127];
tmpQN1[128] = + tmpQN2[128];
tmpQN1[129] = + tmpQN2[129];
tmpQN1[130] = + tmpQN2[130];
tmpQN1[131] = + tmpQN2[131];
tmpQN1[132] = + tmpQN2[132];
tmpQN1[133] = + tmpQN2[133];
tmpQN1[134] = + tmpQN2[134];
tmpQN1[135] = + tmpQN2[135];
tmpQN1[136] = + tmpQN2[136];
tmpQN1[137] = + tmpQN2[137];
tmpQN1[138] = + tmpQN2[138];
tmpQN1[139] = + tmpQN2[139];
tmpQN1[140] = + tmpQN2[140];
tmpQN1[141] = + tmpQN2[141];
tmpQN1[142] = + tmpQN2[142];
tmpQN1[143] = + tmpQN2[143];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 12];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 12 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 12 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 12 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 12 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 12 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 12 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 12 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 12 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 12 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[runObj * 12 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[runObj * 12 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[13] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.u[runObj * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 16] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 16 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 16 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 16 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 16 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 16 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 16 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 16 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 16 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 16 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 16 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 16 + 11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.Dy[runObj * 16 + 12] = acadoWorkspace.objValueOut[12];
acadoWorkspace.Dy[runObj * 16 + 13] = acadoWorkspace.objValueOut[13];
acadoWorkspace.Dy[runObj * 16 + 14] = acadoWorkspace.objValueOut[14];
acadoWorkspace.Dy[runObj * 16 + 15] = acadoWorkspace.objValueOut[15];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 256 ]), &(acadoWorkspace.Q1[ runObj * 144 ]), &(acadoWorkspace.Q2[ runObj * 192 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 256 ]), &(acadoWorkspace.R1[ runObj * 16 ]), &(acadoWorkspace.R2[ runObj * 64 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.x[124];
acadoWorkspace.objValueIn[5] = acadoVariables.x[125];
acadoWorkspace.objValueIn[6] = acadoVariables.x[126];
acadoWorkspace.objValueIn[7] = acadoVariables.x[127];
acadoWorkspace.objValueIn[8] = acadoVariables.x[128];
acadoWorkspace.objValueIn[9] = acadoVariables.x[129];
acadoWorkspace.objValueIn[10] = acadoVariables.x[130];
acadoWorkspace.objValueIn[11] = acadoVariables.x[131];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.DyN[10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.DyN[11] = acadoWorkspace.objValueOut[11];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36] + Gx1[10]*Gu1[40] + Gx1[11]*Gu1[44];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37] + Gx1[10]*Gu1[41] + Gx1[11]*Gu1[45];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38] + Gx1[10]*Gu1[42] + Gx1[11]*Gu1[46];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39] + Gx1[10]*Gu1[43] + Gx1[11]*Gu1[47];
Gu2[4] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[12] + Gx1[16]*Gu1[16] + Gx1[17]*Gu1[20] + Gx1[18]*Gu1[24] + Gx1[19]*Gu1[28] + Gx1[20]*Gu1[32] + Gx1[21]*Gu1[36] + Gx1[22]*Gu1[40] + Gx1[23]*Gu1[44];
Gu2[5] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[9] + Gx1[15]*Gu1[13] + Gx1[16]*Gu1[17] + Gx1[17]*Gu1[21] + Gx1[18]*Gu1[25] + Gx1[19]*Gu1[29] + Gx1[20]*Gu1[33] + Gx1[21]*Gu1[37] + Gx1[22]*Gu1[41] + Gx1[23]*Gu1[45];
Gu2[6] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[10] + Gx1[15]*Gu1[14] + Gx1[16]*Gu1[18] + Gx1[17]*Gu1[22] + Gx1[18]*Gu1[26] + Gx1[19]*Gu1[30] + Gx1[20]*Gu1[34] + Gx1[21]*Gu1[38] + Gx1[22]*Gu1[42] + Gx1[23]*Gu1[46];
Gu2[7] = + Gx1[12]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[11] + Gx1[15]*Gu1[15] + Gx1[16]*Gu1[19] + Gx1[17]*Gu1[23] + Gx1[18]*Gu1[27] + Gx1[19]*Gu1[31] + Gx1[20]*Gu1[35] + Gx1[21]*Gu1[39] + Gx1[22]*Gu1[43] + Gx1[23]*Gu1[47];
Gu2[8] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[27]*Gu1[12] + Gx1[28]*Gu1[16] + Gx1[29]*Gu1[20] + Gx1[30]*Gu1[24] + Gx1[31]*Gu1[28] + Gx1[32]*Gu1[32] + Gx1[33]*Gu1[36] + Gx1[34]*Gu1[40] + Gx1[35]*Gu1[44];
Gu2[9] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[27]*Gu1[13] + Gx1[28]*Gu1[17] + Gx1[29]*Gu1[21] + Gx1[30]*Gu1[25] + Gx1[31]*Gu1[29] + Gx1[32]*Gu1[33] + Gx1[33]*Gu1[37] + Gx1[34]*Gu1[41] + Gx1[35]*Gu1[45];
Gu2[10] = + Gx1[24]*Gu1[2] + Gx1[25]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[27]*Gu1[14] + Gx1[28]*Gu1[18] + Gx1[29]*Gu1[22] + Gx1[30]*Gu1[26] + Gx1[31]*Gu1[30] + Gx1[32]*Gu1[34] + Gx1[33]*Gu1[38] + Gx1[34]*Gu1[42] + Gx1[35]*Gu1[46];
Gu2[11] = + Gx1[24]*Gu1[3] + Gx1[25]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[27]*Gu1[15] + Gx1[28]*Gu1[19] + Gx1[29]*Gu1[23] + Gx1[30]*Gu1[27] + Gx1[31]*Gu1[31] + Gx1[32]*Gu1[35] + Gx1[33]*Gu1[39] + Gx1[34]*Gu1[43] + Gx1[35]*Gu1[47];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[40]*Gu1[16] + Gx1[41]*Gu1[20] + Gx1[42]*Gu1[24] + Gx1[43]*Gu1[28] + Gx1[44]*Gu1[32] + Gx1[45]*Gu1[36] + Gx1[46]*Gu1[40] + Gx1[47]*Gu1[44];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[40]*Gu1[17] + Gx1[41]*Gu1[21] + Gx1[42]*Gu1[25] + Gx1[43]*Gu1[29] + Gx1[44]*Gu1[33] + Gx1[45]*Gu1[37] + Gx1[46]*Gu1[41] + Gx1[47]*Gu1[45];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[6] + Gx1[38]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[40]*Gu1[18] + Gx1[41]*Gu1[22] + Gx1[42]*Gu1[26] + Gx1[43]*Gu1[30] + Gx1[44]*Gu1[34] + Gx1[45]*Gu1[38] + Gx1[46]*Gu1[42] + Gx1[47]*Gu1[46];
Gu2[15] = + Gx1[36]*Gu1[3] + Gx1[37]*Gu1[7] + Gx1[38]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[40]*Gu1[19] + Gx1[41]*Gu1[23] + Gx1[42]*Gu1[27] + Gx1[43]*Gu1[31] + Gx1[44]*Gu1[35] + Gx1[45]*Gu1[39] + Gx1[46]*Gu1[43] + Gx1[47]*Gu1[47];
Gu2[16] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[4] + Gx1[50]*Gu1[8] + Gx1[51]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[54]*Gu1[24] + Gx1[55]*Gu1[28] + Gx1[56]*Gu1[32] + Gx1[57]*Gu1[36] + Gx1[58]*Gu1[40] + Gx1[59]*Gu1[44];
Gu2[17] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[5] + Gx1[50]*Gu1[9] + Gx1[51]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[54]*Gu1[25] + Gx1[55]*Gu1[29] + Gx1[56]*Gu1[33] + Gx1[57]*Gu1[37] + Gx1[58]*Gu1[41] + Gx1[59]*Gu1[45];
Gu2[18] = + Gx1[48]*Gu1[2] + Gx1[49]*Gu1[6] + Gx1[50]*Gu1[10] + Gx1[51]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[54]*Gu1[26] + Gx1[55]*Gu1[30] + Gx1[56]*Gu1[34] + Gx1[57]*Gu1[38] + Gx1[58]*Gu1[42] + Gx1[59]*Gu1[46];
Gu2[19] = + Gx1[48]*Gu1[3] + Gx1[49]*Gu1[7] + Gx1[50]*Gu1[11] + Gx1[51]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[54]*Gu1[27] + Gx1[55]*Gu1[31] + Gx1[56]*Gu1[35] + Gx1[57]*Gu1[39] + Gx1[58]*Gu1[43] + Gx1[59]*Gu1[47];
Gu2[20] = + Gx1[60]*Gu1[0] + Gx1[61]*Gu1[4] + Gx1[62]*Gu1[8] + Gx1[63]*Gu1[12] + Gx1[64]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[67]*Gu1[28] + Gx1[68]*Gu1[32] + Gx1[69]*Gu1[36] + Gx1[70]*Gu1[40] + Gx1[71]*Gu1[44];
Gu2[21] = + Gx1[60]*Gu1[1] + Gx1[61]*Gu1[5] + Gx1[62]*Gu1[9] + Gx1[63]*Gu1[13] + Gx1[64]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[67]*Gu1[29] + Gx1[68]*Gu1[33] + Gx1[69]*Gu1[37] + Gx1[70]*Gu1[41] + Gx1[71]*Gu1[45];
Gu2[22] = + Gx1[60]*Gu1[2] + Gx1[61]*Gu1[6] + Gx1[62]*Gu1[10] + Gx1[63]*Gu1[14] + Gx1[64]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[67]*Gu1[30] + Gx1[68]*Gu1[34] + Gx1[69]*Gu1[38] + Gx1[70]*Gu1[42] + Gx1[71]*Gu1[46];
Gu2[23] = + Gx1[60]*Gu1[3] + Gx1[61]*Gu1[7] + Gx1[62]*Gu1[11] + Gx1[63]*Gu1[15] + Gx1[64]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[67]*Gu1[31] + Gx1[68]*Gu1[35] + Gx1[69]*Gu1[39] + Gx1[70]*Gu1[43] + Gx1[71]*Gu1[47];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[12] + Gx1[76]*Gu1[16] + Gx1[77]*Gu1[20] + Gx1[78]*Gu1[24] + Gx1[79]*Gu1[28] + Gx1[80]*Gu1[32] + Gx1[81]*Gu1[36] + Gx1[82]*Gu1[40] + Gx1[83]*Gu1[44];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[9] + Gx1[75]*Gu1[13] + Gx1[76]*Gu1[17] + Gx1[77]*Gu1[21] + Gx1[78]*Gu1[25] + Gx1[79]*Gu1[29] + Gx1[80]*Gu1[33] + Gx1[81]*Gu1[37] + Gx1[82]*Gu1[41] + Gx1[83]*Gu1[45];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[6] + Gx1[74]*Gu1[10] + Gx1[75]*Gu1[14] + Gx1[76]*Gu1[18] + Gx1[77]*Gu1[22] + Gx1[78]*Gu1[26] + Gx1[79]*Gu1[30] + Gx1[80]*Gu1[34] + Gx1[81]*Gu1[38] + Gx1[82]*Gu1[42] + Gx1[83]*Gu1[46];
Gu2[27] = + Gx1[72]*Gu1[3] + Gx1[73]*Gu1[7] + Gx1[74]*Gu1[11] + Gx1[75]*Gu1[15] + Gx1[76]*Gu1[19] + Gx1[77]*Gu1[23] + Gx1[78]*Gu1[27] + Gx1[79]*Gu1[31] + Gx1[80]*Gu1[35] + Gx1[81]*Gu1[39] + Gx1[82]*Gu1[43] + Gx1[83]*Gu1[47];
Gu2[28] = + Gx1[84]*Gu1[0] + Gx1[85]*Gu1[4] + Gx1[86]*Gu1[8] + Gx1[87]*Gu1[12] + Gx1[88]*Gu1[16] + Gx1[89]*Gu1[20] + Gx1[90]*Gu1[24] + Gx1[91]*Gu1[28] + Gx1[92]*Gu1[32] + Gx1[93]*Gu1[36] + Gx1[94]*Gu1[40] + Gx1[95]*Gu1[44];
Gu2[29] = + Gx1[84]*Gu1[1] + Gx1[85]*Gu1[5] + Gx1[86]*Gu1[9] + Gx1[87]*Gu1[13] + Gx1[88]*Gu1[17] + Gx1[89]*Gu1[21] + Gx1[90]*Gu1[25] + Gx1[91]*Gu1[29] + Gx1[92]*Gu1[33] + Gx1[93]*Gu1[37] + Gx1[94]*Gu1[41] + Gx1[95]*Gu1[45];
Gu2[30] = + Gx1[84]*Gu1[2] + Gx1[85]*Gu1[6] + Gx1[86]*Gu1[10] + Gx1[87]*Gu1[14] + Gx1[88]*Gu1[18] + Gx1[89]*Gu1[22] + Gx1[90]*Gu1[26] + Gx1[91]*Gu1[30] + Gx1[92]*Gu1[34] + Gx1[93]*Gu1[38] + Gx1[94]*Gu1[42] + Gx1[95]*Gu1[46];
Gu2[31] = + Gx1[84]*Gu1[3] + Gx1[85]*Gu1[7] + Gx1[86]*Gu1[11] + Gx1[87]*Gu1[15] + Gx1[88]*Gu1[19] + Gx1[89]*Gu1[23] + Gx1[90]*Gu1[27] + Gx1[91]*Gu1[31] + Gx1[92]*Gu1[35] + Gx1[93]*Gu1[39] + Gx1[94]*Gu1[43] + Gx1[95]*Gu1[47];
Gu2[32] = + Gx1[96]*Gu1[0] + Gx1[97]*Gu1[4] + Gx1[98]*Gu1[8] + Gx1[99]*Gu1[12] + Gx1[100]*Gu1[16] + Gx1[101]*Gu1[20] + Gx1[102]*Gu1[24] + Gx1[103]*Gu1[28] + Gx1[104]*Gu1[32] + Gx1[105]*Gu1[36] + Gx1[106]*Gu1[40] + Gx1[107]*Gu1[44];
Gu2[33] = + Gx1[96]*Gu1[1] + Gx1[97]*Gu1[5] + Gx1[98]*Gu1[9] + Gx1[99]*Gu1[13] + Gx1[100]*Gu1[17] + Gx1[101]*Gu1[21] + Gx1[102]*Gu1[25] + Gx1[103]*Gu1[29] + Gx1[104]*Gu1[33] + Gx1[105]*Gu1[37] + Gx1[106]*Gu1[41] + Gx1[107]*Gu1[45];
Gu2[34] = + Gx1[96]*Gu1[2] + Gx1[97]*Gu1[6] + Gx1[98]*Gu1[10] + Gx1[99]*Gu1[14] + Gx1[100]*Gu1[18] + Gx1[101]*Gu1[22] + Gx1[102]*Gu1[26] + Gx1[103]*Gu1[30] + Gx1[104]*Gu1[34] + Gx1[105]*Gu1[38] + Gx1[106]*Gu1[42] + Gx1[107]*Gu1[46];
Gu2[35] = + Gx1[96]*Gu1[3] + Gx1[97]*Gu1[7] + Gx1[98]*Gu1[11] + Gx1[99]*Gu1[15] + Gx1[100]*Gu1[19] + Gx1[101]*Gu1[23] + Gx1[102]*Gu1[27] + Gx1[103]*Gu1[31] + Gx1[104]*Gu1[35] + Gx1[105]*Gu1[39] + Gx1[106]*Gu1[43] + Gx1[107]*Gu1[47];
Gu2[36] = + Gx1[108]*Gu1[0] + Gx1[109]*Gu1[4] + Gx1[110]*Gu1[8] + Gx1[111]*Gu1[12] + Gx1[112]*Gu1[16] + Gx1[113]*Gu1[20] + Gx1[114]*Gu1[24] + Gx1[115]*Gu1[28] + Gx1[116]*Gu1[32] + Gx1[117]*Gu1[36] + Gx1[118]*Gu1[40] + Gx1[119]*Gu1[44];
Gu2[37] = + Gx1[108]*Gu1[1] + Gx1[109]*Gu1[5] + Gx1[110]*Gu1[9] + Gx1[111]*Gu1[13] + Gx1[112]*Gu1[17] + Gx1[113]*Gu1[21] + Gx1[114]*Gu1[25] + Gx1[115]*Gu1[29] + Gx1[116]*Gu1[33] + Gx1[117]*Gu1[37] + Gx1[118]*Gu1[41] + Gx1[119]*Gu1[45];
Gu2[38] = + Gx1[108]*Gu1[2] + Gx1[109]*Gu1[6] + Gx1[110]*Gu1[10] + Gx1[111]*Gu1[14] + Gx1[112]*Gu1[18] + Gx1[113]*Gu1[22] + Gx1[114]*Gu1[26] + Gx1[115]*Gu1[30] + Gx1[116]*Gu1[34] + Gx1[117]*Gu1[38] + Gx1[118]*Gu1[42] + Gx1[119]*Gu1[46];
Gu2[39] = + Gx1[108]*Gu1[3] + Gx1[109]*Gu1[7] + Gx1[110]*Gu1[11] + Gx1[111]*Gu1[15] + Gx1[112]*Gu1[19] + Gx1[113]*Gu1[23] + Gx1[114]*Gu1[27] + Gx1[115]*Gu1[31] + Gx1[116]*Gu1[35] + Gx1[117]*Gu1[39] + Gx1[118]*Gu1[43] + Gx1[119]*Gu1[47];
Gu2[40] = + Gx1[120]*Gu1[0] + Gx1[121]*Gu1[4] + Gx1[122]*Gu1[8] + Gx1[123]*Gu1[12] + Gx1[124]*Gu1[16] + Gx1[125]*Gu1[20] + Gx1[126]*Gu1[24] + Gx1[127]*Gu1[28] + Gx1[128]*Gu1[32] + Gx1[129]*Gu1[36] + Gx1[130]*Gu1[40] + Gx1[131]*Gu1[44];
Gu2[41] = + Gx1[120]*Gu1[1] + Gx1[121]*Gu1[5] + Gx1[122]*Gu1[9] + Gx1[123]*Gu1[13] + Gx1[124]*Gu1[17] + Gx1[125]*Gu1[21] + Gx1[126]*Gu1[25] + Gx1[127]*Gu1[29] + Gx1[128]*Gu1[33] + Gx1[129]*Gu1[37] + Gx1[130]*Gu1[41] + Gx1[131]*Gu1[45];
Gu2[42] = + Gx1[120]*Gu1[2] + Gx1[121]*Gu1[6] + Gx1[122]*Gu1[10] + Gx1[123]*Gu1[14] + Gx1[124]*Gu1[18] + Gx1[125]*Gu1[22] + Gx1[126]*Gu1[26] + Gx1[127]*Gu1[30] + Gx1[128]*Gu1[34] + Gx1[129]*Gu1[38] + Gx1[130]*Gu1[42] + Gx1[131]*Gu1[46];
Gu2[43] = + Gx1[120]*Gu1[3] + Gx1[121]*Gu1[7] + Gx1[122]*Gu1[11] + Gx1[123]*Gu1[15] + Gx1[124]*Gu1[19] + Gx1[125]*Gu1[23] + Gx1[126]*Gu1[27] + Gx1[127]*Gu1[31] + Gx1[128]*Gu1[35] + Gx1[129]*Gu1[39] + Gx1[130]*Gu1[43] + Gx1[131]*Gu1[47];
Gu2[44] = + Gx1[132]*Gu1[0] + Gx1[133]*Gu1[4] + Gx1[134]*Gu1[8] + Gx1[135]*Gu1[12] + Gx1[136]*Gu1[16] + Gx1[137]*Gu1[20] + Gx1[138]*Gu1[24] + Gx1[139]*Gu1[28] + Gx1[140]*Gu1[32] + Gx1[141]*Gu1[36] + Gx1[142]*Gu1[40] + Gx1[143]*Gu1[44];
Gu2[45] = + Gx1[132]*Gu1[1] + Gx1[133]*Gu1[5] + Gx1[134]*Gu1[9] + Gx1[135]*Gu1[13] + Gx1[136]*Gu1[17] + Gx1[137]*Gu1[21] + Gx1[138]*Gu1[25] + Gx1[139]*Gu1[29] + Gx1[140]*Gu1[33] + Gx1[141]*Gu1[37] + Gx1[142]*Gu1[41] + Gx1[143]*Gu1[45];
Gu2[46] = + Gx1[132]*Gu1[2] + Gx1[133]*Gu1[6] + Gx1[134]*Gu1[10] + Gx1[135]*Gu1[14] + Gx1[136]*Gu1[18] + Gx1[137]*Gu1[22] + Gx1[138]*Gu1[26] + Gx1[139]*Gu1[30] + Gx1[140]*Gu1[34] + Gx1[141]*Gu1[38] + Gx1[142]*Gu1[42] + Gx1[143]*Gu1[46];
Gu2[47] = + Gx1[132]*Gu1[3] + Gx1[133]*Gu1[7] + Gx1[134]*Gu1[11] + Gx1[135]*Gu1[15] + Gx1[136]*Gu1[19] + Gx1[137]*Gu1[23] + Gx1[138]*Gu1[27] + Gx1[139]*Gu1[31] + Gx1[140]*Gu1[35] + Gx1[141]*Gu1[39] + Gx1[142]*Gu1[43] + Gx1[143]*Gu1[47];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
Gu2[40] = Gu1[40];
Gu2[41] = Gu1[41];
Gu2[42] = Gu1[42];
Gu2[43] = Gu1[43];
Gu2[44] = Gu1[44];
Gu2[45] = Gu1[45];
Gu2[46] = Gu1[46];
Gu2[47] = Gu1[47];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 160) + (iCol * 4)] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44];
acadoWorkspace.H[(iRow * 160) + (iCol * 4 + 1)] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45];
acadoWorkspace.H[(iRow * 160) + (iCol * 4 + 2)] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46];
acadoWorkspace.H[(iRow * 160) + (iCol * 4 + 3)] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47];
acadoWorkspace.H[(iRow * 160 + 40) + (iCol * 4)] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44];
acadoWorkspace.H[(iRow * 160 + 40) + (iCol * 4 + 1)] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45];
acadoWorkspace.H[(iRow * 160 + 40) + (iCol * 4 + 2)] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46];
acadoWorkspace.H[(iRow * 160 + 40) + (iCol * 4 + 3)] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47];
acadoWorkspace.H[(iRow * 160 + 80) + (iCol * 4)] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44];
acadoWorkspace.H[(iRow * 160 + 80) + (iCol * 4 + 1)] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45];
acadoWorkspace.H[(iRow * 160 + 80) + (iCol * 4 + 2)] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46];
acadoWorkspace.H[(iRow * 160 + 80) + (iCol * 4 + 3)] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47];
acadoWorkspace.H[(iRow * 160 + 120) + (iCol * 4)] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44];
acadoWorkspace.H[(iRow * 160 + 120) + (iCol * 4 + 1)] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45];
acadoWorkspace.H[(iRow * 160 + 120) + (iCol * 4 + 2)] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46];
acadoWorkspace.H[(iRow * 160 + 120) + (iCol * 4 + 3)] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 164] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44] + R11[0];
acadoWorkspace.H[iRow * 164 + 1] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45] + R11[1];
acadoWorkspace.H[iRow * 164 + 2] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46] + R11[2];
acadoWorkspace.H[iRow * 164 + 3] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47] + R11[3];
acadoWorkspace.H[iRow * 164 + 40] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44] + R11[4];
acadoWorkspace.H[iRow * 164 + 41] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45] + R11[5];
acadoWorkspace.H[iRow * 164 + 42] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46] + R11[6];
acadoWorkspace.H[iRow * 164 + 43] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47] + R11[7];
acadoWorkspace.H[iRow * 164 + 80] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44] + R11[8];
acadoWorkspace.H[iRow * 164 + 81] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45] + R11[9];
acadoWorkspace.H[iRow * 164 + 82] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46] + R11[10];
acadoWorkspace.H[iRow * 164 + 83] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47] + R11[11];
acadoWorkspace.H[iRow * 164 + 120] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44] + R11[12];
acadoWorkspace.H[iRow * 164 + 121] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45] + R11[13];
acadoWorkspace.H[iRow * 164 + 122] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46] + R11[14];
acadoWorkspace.H[iRow * 164 + 123] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47] + R11[15];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[12]*Gu1[4] + Gx1[24]*Gu1[8] + Gx1[36]*Gu1[12] + Gx1[48]*Gu1[16] + Gx1[60]*Gu1[20] + Gx1[72]*Gu1[24] + Gx1[84]*Gu1[28] + Gx1[96]*Gu1[32] + Gx1[108]*Gu1[36] + Gx1[120]*Gu1[40] + Gx1[132]*Gu1[44];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[12]*Gu1[5] + Gx1[24]*Gu1[9] + Gx1[36]*Gu1[13] + Gx1[48]*Gu1[17] + Gx1[60]*Gu1[21] + Gx1[72]*Gu1[25] + Gx1[84]*Gu1[29] + Gx1[96]*Gu1[33] + Gx1[108]*Gu1[37] + Gx1[120]*Gu1[41] + Gx1[132]*Gu1[45];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[12]*Gu1[6] + Gx1[24]*Gu1[10] + Gx1[36]*Gu1[14] + Gx1[48]*Gu1[18] + Gx1[60]*Gu1[22] + Gx1[72]*Gu1[26] + Gx1[84]*Gu1[30] + Gx1[96]*Gu1[34] + Gx1[108]*Gu1[38] + Gx1[120]*Gu1[42] + Gx1[132]*Gu1[46];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[12]*Gu1[7] + Gx1[24]*Gu1[11] + Gx1[36]*Gu1[15] + Gx1[48]*Gu1[19] + Gx1[60]*Gu1[23] + Gx1[72]*Gu1[27] + Gx1[84]*Gu1[31] + Gx1[96]*Gu1[35] + Gx1[108]*Gu1[39] + Gx1[120]*Gu1[43] + Gx1[132]*Gu1[47];
Gu2[4] = + Gx1[1]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[25]*Gu1[8] + Gx1[37]*Gu1[12] + Gx1[49]*Gu1[16] + Gx1[61]*Gu1[20] + Gx1[73]*Gu1[24] + Gx1[85]*Gu1[28] + Gx1[97]*Gu1[32] + Gx1[109]*Gu1[36] + Gx1[121]*Gu1[40] + Gx1[133]*Gu1[44];
Gu2[5] = + Gx1[1]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[25]*Gu1[9] + Gx1[37]*Gu1[13] + Gx1[49]*Gu1[17] + Gx1[61]*Gu1[21] + Gx1[73]*Gu1[25] + Gx1[85]*Gu1[29] + Gx1[97]*Gu1[33] + Gx1[109]*Gu1[37] + Gx1[121]*Gu1[41] + Gx1[133]*Gu1[45];
Gu2[6] = + Gx1[1]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[25]*Gu1[10] + Gx1[37]*Gu1[14] + Gx1[49]*Gu1[18] + Gx1[61]*Gu1[22] + Gx1[73]*Gu1[26] + Gx1[85]*Gu1[30] + Gx1[97]*Gu1[34] + Gx1[109]*Gu1[38] + Gx1[121]*Gu1[42] + Gx1[133]*Gu1[46];
Gu2[7] = + Gx1[1]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[25]*Gu1[11] + Gx1[37]*Gu1[15] + Gx1[49]*Gu1[19] + Gx1[61]*Gu1[23] + Gx1[73]*Gu1[27] + Gx1[85]*Gu1[31] + Gx1[97]*Gu1[35] + Gx1[109]*Gu1[39] + Gx1[121]*Gu1[43] + Gx1[133]*Gu1[47];
Gu2[8] = + Gx1[2]*Gu1[0] + Gx1[14]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[38]*Gu1[12] + Gx1[50]*Gu1[16] + Gx1[62]*Gu1[20] + Gx1[74]*Gu1[24] + Gx1[86]*Gu1[28] + Gx1[98]*Gu1[32] + Gx1[110]*Gu1[36] + Gx1[122]*Gu1[40] + Gx1[134]*Gu1[44];
Gu2[9] = + Gx1[2]*Gu1[1] + Gx1[14]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[38]*Gu1[13] + Gx1[50]*Gu1[17] + Gx1[62]*Gu1[21] + Gx1[74]*Gu1[25] + Gx1[86]*Gu1[29] + Gx1[98]*Gu1[33] + Gx1[110]*Gu1[37] + Gx1[122]*Gu1[41] + Gx1[134]*Gu1[45];
Gu2[10] = + Gx1[2]*Gu1[2] + Gx1[14]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[38]*Gu1[14] + Gx1[50]*Gu1[18] + Gx1[62]*Gu1[22] + Gx1[74]*Gu1[26] + Gx1[86]*Gu1[30] + Gx1[98]*Gu1[34] + Gx1[110]*Gu1[38] + Gx1[122]*Gu1[42] + Gx1[134]*Gu1[46];
Gu2[11] = + Gx1[2]*Gu1[3] + Gx1[14]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[38]*Gu1[15] + Gx1[50]*Gu1[19] + Gx1[62]*Gu1[23] + Gx1[74]*Gu1[27] + Gx1[86]*Gu1[31] + Gx1[98]*Gu1[35] + Gx1[110]*Gu1[39] + Gx1[122]*Gu1[43] + Gx1[134]*Gu1[47];
Gu2[12] = + Gx1[3]*Gu1[0] + Gx1[15]*Gu1[4] + Gx1[27]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[51]*Gu1[16] + Gx1[63]*Gu1[20] + Gx1[75]*Gu1[24] + Gx1[87]*Gu1[28] + Gx1[99]*Gu1[32] + Gx1[111]*Gu1[36] + Gx1[123]*Gu1[40] + Gx1[135]*Gu1[44];
Gu2[13] = + Gx1[3]*Gu1[1] + Gx1[15]*Gu1[5] + Gx1[27]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[51]*Gu1[17] + Gx1[63]*Gu1[21] + Gx1[75]*Gu1[25] + Gx1[87]*Gu1[29] + Gx1[99]*Gu1[33] + Gx1[111]*Gu1[37] + Gx1[123]*Gu1[41] + Gx1[135]*Gu1[45];
Gu2[14] = + Gx1[3]*Gu1[2] + Gx1[15]*Gu1[6] + Gx1[27]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[51]*Gu1[18] + Gx1[63]*Gu1[22] + Gx1[75]*Gu1[26] + Gx1[87]*Gu1[30] + Gx1[99]*Gu1[34] + Gx1[111]*Gu1[38] + Gx1[123]*Gu1[42] + Gx1[135]*Gu1[46];
Gu2[15] = + Gx1[3]*Gu1[3] + Gx1[15]*Gu1[7] + Gx1[27]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[51]*Gu1[19] + Gx1[63]*Gu1[23] + Gx1[75]*Gu1[27] + Gx1[87]*Gu1[31] + Gx1[99]*Gu1[35] + Gx1[111]*Gu1[39] + Gx1[123]*Gu1[43] + Gx1[135]*Gu1[47];
Gu2[16] = + Gx1[4]*Gu1[0] + Gx1[16]*Gu1[4] + Gx1[28]*Gu1[8] + Gx1[40]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[64]*Gu1[20] + Gx1[76]*Gu1[24] + Gx1[88]*Gu1[28] + Gx1[100]*Gu1[32] + Gx1[112]*Gu1[36] + Gx1[124]*Gu1[40] + Gx1[136]*Gu1[44];
Gu2[17] = + Gx1[4]*Gu1[1] + Gx1[16]*Gu1[5] + Gx1[28]*Gu1[9] + Gx1[40]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[64]*Gu1[21] + Gx1[76]*Gu1[25] + Gx1[88]*Gu1[29] + Gx1[100]*Gu1[33] + Gx1[112]*Gu1[37] + Gx1[124]*Gu1[41] + Gx1[136]*Gu1[45];
Gu2[18] = + Gx1[4]*Gu1[2] + Gx1[16]*Gu1[6] + Gx1[28]*Gu1[10] + Gx1[40]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[64]*Gu1[22] + Gx1[76]*Gu1[26] + Gx1[88]*Gu1[30] + Gx1[100]*Gu1[34] + Gx1[112]*Gu1[38] + Gx1[124]*Gu1[42] + Gx1[136]*Gu1[46];
Gu2[19] = + Gx1[4]*Gu1[3] + Gx1[16]*Gu1[7] + Gx1[28]*Gu1[11] + Gx1[40]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[64]*Gu1[23] + Gx1[76]*Gu1[27] + Gx1[88]*Gu1[31] + Gx1[100]*Gu1[35] + Gx1[112]*Gu1[39] + Gx1[124]*Gu1[43] + Gx1[136]*Gu1[47];
Gu2[20] = + Gx1[5]*Gu1[0] + Gx1[17]*Gu1[4] + Gx1[29]*Gu1[8] + Gx1[41]*Gu1[12] + Gx1[53]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[77]*Gu1[24] + Gx1[89]*Gu1[28] + Gx1[101]*Gu1[32] + Gx1[113]*Gu1[36] + Gx1[125]*Gu1[40] + Gx1[137]*Gu1[44];
Gu2[21] = + Gx1[5]*Gu1[1] + Gx1[17]*Gu1[5] + Gx1[29]*Gu1[9] + Gx1[41]*Gu1[13] + Gx1[53]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[77]*Gu1[25] + Gx1[89]*Gu1[29] + Gx1[101]*Gu1[33] + Gx1[113]*Gu1[37] + Gx1[125]*Gu1[41] + Gx1[137]*Gu1[45];
Gu2[22] = + Gx1[5]*Gu1[2] + Gx1[17]*Gu1[6] + Gx1[29]*Gu1[10] + Gx1[41]*Gu1[14] + Gx1[53]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[77]*Gu1[26] + Gx1[89]*Gu1[30] + Gx1[101]*Gu1[34] + Gx1[113]*Gu1[38] + Gx1[125]*Gu1[42] + Gx1[137]*Gu1[46];
Gu2[23] = + Gx1[5]*Gu1[3] + Gx1[17]*Gu1[7] + Gx1[29]*Gu1[11] + Gx1[41]*Gu1[15] + Gx1[53]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[77]*Gu1[27] + Gx1[89]*Gu1[31] + Gx1[101]*Gu1[35] + Gx1[113]*Gu1[39] + Gx1[125]*Gu1[43] + Gx1[137]*Gu1[47];
Gu2[24] = + Gx1[6]*Gu1[0] + Gx1[18]*Gu1[4] + Gx1[30]*Gu1[8] + Gx1[42]*Gu1[12] + Gx1[54]*Gu1[16] + Gx1[66]*Gu1[20] + Gx1[78]*Gu1[24] + Gx1[90]*Gu1[28] + Gx1[102]*Gu1[32] + Gx1[114]*Gu1[36] + Gx1[126]*Gu1[40] + Gx1[138]*Gu1[44];
Gu2[25] = + Gx1[6]*Gu1[1] + Gx1[18]*Gu1[5] + Gx1[30]*Gu1[9] + Gx1[42]*Gu1[13] + Gx1[54]*Gu1[17] + Gx1[66]*Gu1[21] + Gx1[78]*Gu1[25] + Gx1[90]*Gu1[29] + Gx1[102]*Gu1[33] + Gx1[114]*Gu1[37] + Gx1[126]*Gu1[41] + Gx1[138]*Gu1[45];
Gu2[26] = + Gx1[6]*Gu1[2] + Gx1[18]*Gu1[6] + Gx1[30]*Gu1[10] + Gx1[42]*Gu1[14] + Gx1[54]*Gu1[18] + Gx1[66]*Gu1[22] + Gx1[78]*Gu1[26] + Gx1[90]*Gu1[30] + Gx1[102]*Gu1[34] + Gx1[114]*Gu1[38] + Gx1[126]*Gu1[42] + Gx1[138]*Gu1[46];
Gu2[27] = + Gx1[6]*Gu1[3] + Gx1[18]*Gu1[7] + Gx1[30]*Gu1[11] + Gx1[42]*Gu1[15] + Gx1[54]*Gu1[19] + Gx1[66]*Gu1[23] + Gx1[78]*Gu1[27] + Gx1[90]*Gu1[31] + Gx1[102]*Gu1[35] + Gx1[114]*Gu1[39] + Gx1[126]*Gu1[43] + Gx1[138]*Gu1[47];
Gu2[28] = + Gx1[7]*Gu1[0] + Gx1[19]*Gu1[4] + Gx1[31]*Gu1[8] + Gx1[43]*Gu1[12] + Gx1[55]*Gu1[16] + Gx1[67]*Gu1[20] + Gx1[79]*Gu1[24] + Gx1[91]*Gu1[28] + Gx1[103]*Gu1[32] + Gx1[115]*Gu1[36] + Gx1[127]*Gu1[40] + Gx1[139]*Gu1[44];
Gu2[29] = + Gx1[7]*Gu1[1] + Gx1[19]*Gu1[5] + Gx1[31]*Gu1[9] + Gx1[43]*Gu1[13] + Gx1[55]*Gu1[17] + Gx1[67]*Gu1[21] + Gx1[79]*Gu1[25] + Gx1[91]*Gu1[29] + Gx1[103]*Gu1[33] + Gx1[115]*Gu1[37] + Gx1[127]*Gu1[41] + Gx1[139]*Gu1[45];
Gu2[30] = + Gx1[7]*Gu1[2] + Gx1[19]*Gu1[6] + Gx1[31]*Gu1[10] + Gx1[43]*Gu1[14] + Gx1[55]*Gu1[18] + Gx1[67]*Gu1[22] + Gx1[79]*Gu1[26] + Gx1[91]*Gu1[30] + Gx1[103]*Gu1[34] + Gx1[115]*Gu1[38] + Gx1[127]*Gu1[42] + Gx1[139]*Gu1[46];
Gu2[31] = + Gx1[7]*Gu1[3] + Gx1[19]*Gu1[7] + Gx1[31]*Gu1[11] + Gx1[43]*Gu1[15] + Gx1[55]*Gu1[19] + Gx1[67]*Gu1[23] + Gx1[79]*Gu1[27] + Gx1[91]*Gu1[31] + Gx1[103]*Gu1[35] + Gx1[115]*Gu1[39] + Gx1[127]*Gu1[43] + Gx1[139]*Gu1[47];
Gu2[32] = + Gx1[8]*Gu1[0] + Gx1[20]*Gu1[4] + Gx1[32]*Gu1[8] + Gx1[44]*Gu1[12] + Gx1[56]*Gu1[16] + Gx1[68]*Gu1[20] + Gx1[80]*Gu1[24] + Gx1[92]*Gu1[28] + Gx1[104]*Gu1[32] + Gx1[116]*Gu1[36] + Gx1[128]*Gu1[40] + Gx1[140]*Gu1[44];
Gu2[33] = + Gx1[8]*Gu1[1] + Gx1[20]*Gu1[5] + Gx1[32]*Gu1[9] + Gx1[44]*Gu1[13] + Gx1[56]*Gu1[17] + Gx1[68]*Gu1[21] + Gx1[80]*Gu1[25] + Gx1[92]*Gu1[29] + Gx1[104]*Gu1[33] + Gx1[116]*Gu1[37] + Gx1[128]*Gu1[41] + Gx1[140]*Gu1[45];
Gu2[34] = + Gx1[8]*Gu1[2] + Gx1[20]*Gu1[6] + Gx1[32]*Gu1[10] + Gx1[44]*Gu1[14] + Gx1[56]*Gu1[18] + Gx1[68]*Gu1[22] + Gx1[80]*Gu1[26] + Gx1[92]*Gu1[30] + Gx1[104]*Gu1[34] + Gx1[116]*Gu1[38] + Gx1[128]*Gu1[42] + Gx1[140]*Gu1[46];
Gu2[35] = + Gx1[8]*Gu1[3] + Gx1[20]*Gu1[7] + Gx1[32]*Gu1[11] + Gx1[44]*Gu1[15] + Gx1[56]*Gu1[19] + Gx1[68]*Gu1[23] + Gx1[80]*Gu1[27] + Gx1[92]*Gu1[31] + Gx1[104]*Gu1[35] + Gx1[116]*Gu1[39] + Gx1[128]*Gu1[43] + Gx1[140]*Gu1[47];
Gu2[36] = + Gx1[9]*Gu1[0] + Gx1[21]*Gu1[4] + Gx1[33]*Gu1[8] + Gx1[45]*Gu1[12] + Gx1[57]*Gu1[16] + Gx1[69]*Gu1[20] + Gx1[81]*Gu1[24] + Gx1[93]*Gu1[28] + Gx1[105]*Gu1[32] + Gx1[117]*Gu1[36] + Gx1[129]*Gu1[40] + Gx1[141]*Gu1[44];
Gu2[37] = + Gx1[9]*Gu1[1] + Gx1[21]*Gu1[5] + Gx1[33]*Gu1[9] + Gx1[45]*Gu1[13] + Gx1[57]*Gu1[17] + Gx1[69]*Gu1[21] + Gx1[81]*Gu1[25] + Gx1[93]*Gu1[29] + Gx1[105]*Gu1[33] + Gx1[117]*Gu1[37] + Gx1[129]*Gu1[41] + Gx1[141]*Gu1[45];
Gu2[38] = + Gx1[9]*Gu1[2] + Gx1[21]*Gu1[6] + Gx1[33]*Gu1[10] + Gx1[45]*Gu1[14] + Gx1[57]*Gu1[18] + Gx1[69]*Gu1[22] + Gx1[81]*Gu1[26] + Gx1[93]*Gu1[30] + Gx1[105]*Gu1[34] + Gx1[117]*Gu1[38] + Gx1[129]*Gu1[42] + Gx1[141]*Gu1[46];
Gu2[39] = + Gx1[9]*Gu1[3] + Gx1[21]*Gu1[7] + Gx1[33]*Gu1[11] + Gx1[45]*Gu1[15] + Gx1[57]*Gu1[19] + Gx1[69]*Gu1[23] + Gx1[81]*Gu1[27] + Gx1[93]*Gu1[31] + Gx1[105]*Gu1[35] + Gx1[117]*Gu1[39] + Gx1[129]*Gu1[43] + Gx1[141]*Gu1[47];
Gu2[40] = + Gx1[10]*Gu1[0] + Gx1[22]*Gu1[4] + Gx1[34]*Gu1[8] + Gx1[46]*Gu1[12] + Gx1[58]*Gu1[16] + Gx1[70]*Gu1[20] + Gx1[82]*Gu1[24] + Gx1[94]*Gu1[28] + Gx1[106]*Gu1[32] + Gx1[118]*Gu1[36] + Gx1[130]*Gu1[40] + Gx1[142]*Gu1[44];
Gu2[41] = + Gx1[10]*Gu1[1] + Gx1[22]*Gu1[5] + Gx1[34]*Gu1[9] + Gx1[46]*Gu1[13] + Gx1[58]*Gu1[17] + Gx1[70]*Gu1[21] + Gx1[82]*Gu1[25] + Gx1[94]*Gu1[29] + Gx1[106]*Gu1[33] + Gx1[118]*Gu1[37] + Gx1[130]*Gu1[41] + Gx1[142]*Gu1[45];
Gu2[42] = + Gx1[10]*Gu1[2] + Gx1[22]*Gu1[6] + Gx1[34]*Gu1[10] + Gx1[46]*Gu1[14] + Gx1[58]*Gu1[18] + Gx1[70]*Gu1[22] + Gx1[82]*Gu1[26] + Gx1[94]*Gu1[30] + Gx1[106]*Gu1[34] + Gx1[118]*Gu1[38] + Gx1[130]*Gu1[42] + Gx1[142]*Gu1[46];
Gu2[43] = + Gx1[10]*Gu1[3] + Gx1[22]*Gu1[7] + Gx1[34]*Gu1[11] + Gx1[46]*Gu1[15] + Gx1[58]*Gu1[19] + Gx1[70]*Gu1[23] + Gx1[82]*Gu1[27] + Gx1[94]*Gu1[31] + Gx1[106]*Gu1[35] + Gx1[118]*Gu1[39] + Gx1[130]*Gu1[43] + Gx1[142]*Gu1[47];
Gu2[44] = + Gx1[11]*Gu1[0] + Gx1[23]*Gu1[4] + Gx1[35]*Gu1[8] + Gx1[47]*Gu1[12] + Gx1[59]*Gu1[16] + Gx1[71]*Gu1[20] + Gx1[83]*Gu1[24] + Gx1[95]*Gu1[28] + Gx1[107]*Gu1[32] + Gx1[119]*Gu1[36] + Gx1[131]*Gu1[40] + Gx1[143]*Gu1[44];
Gu2[45] = + Gx1[11]*Gu1[1] + Gx1[23]*Gu1[5] + Gx1[35]*Gu1[9] + Gx1[47]*Gu1[13] + Gx1[59]*Gu1[17] + Gx1[71]*Gu1[21] + Gx1[83]*Gu1[25] + Gx1[95]*Gu1[29] + Gx1[107]*Gu1[33] + Gx1[119]*Gu1[37] + Gx1[131]*Gu1[41] + Gx1[143]*Gu1[45];
Gu2[46] = + Gx1[11]*Gu1[2] + Gx1[23]*Gu1[6] + Gx1[35]*Gu1[10] + Gx1[47]*Gu1[14] + Gx1[59]*Gu1[18] + Gx1[71]*Gu1[22] + Gx1[83]*Gu1[26] + Gx1[95]*Gu1[30] + Gx1[107]*Gu1[34] + Gx1[119]*Gu1[38] + Gx1[131]*Gu1[42] + Gx1[143]*Gu1[46];
Gu2[47] = + Gx1[11]*Gu1[3] + Gx1[23]*Gu1[7] + Gx1[35]*Gu1[11] + Gx1[47]*Gu1[15] + Gx1[59]*Gu1[19] + Gx1[71]*Gu1[23] + Gx1[83]*Gu1[27] + Gx1[95]*Gu1[31] + Gx1[107]*Gu1[35] + Gx1[119]*Gu1[39] + Gx1[131]*Gu1[43] + Gx1[143]*Gu1[47];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[4] + Q11[2]*Gu1[8] + Q11[3]*Gu1[12] + Q11[4]*Gu1[16] + Q11[5]*Gu1[20] + Q11[6]*Gu1[24] + Q11[7]*Gu1[28] + Q11[8]*Gu1[32] + Q11[9]*Gu1[36] + Q11[10]*Gu1[40] + Q11[11]*Gu1[44] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[5] + Q11[2]*Gu1[9] + Q11[3]*Gu1[13] + Q11[4]*Gu1[17] + Q11[5]*Gu1[21] + Q11[6]*Gu1[25] + Q11[7]*Gu1[29] + Q11[8]*Gu1[33] + Q11[9]*Gu1[37] + Q11[10]*Gu1[41] + Q11[11]*Gu1[45] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[6] + Q11[2]*Gu1[10] + Q11[3]*Gu1[14] + Q11[4]*Gu1[18] + Q11[5]*Gu1[22] + Q11[6]*Gu1[26] + Q11[7]*Gu1[30] + Q11[8]*Gu1[34] + Q11[9]*Gu1[38] + Q11[10]*Gu1[42] + Q11[11]*Gu1[46] + Gu2[2];
Gu3[3] = + Q11[0]*Gu1[3] + Q11[1]*Gu1[7] + Q11[2]*Gu1[11] + Q11[3]*Gu1[15] + Q11[4]*Gu1[19] + Q11[5]*Gu1[23] + Q11[6]*Gu1[27] + Q11[7]*Gu1[31] + Q11[8]*Gu1[35] + Q11[9]*Gu1[39] + Q11[10]*Gu1[43] + Q11[11]*Gu1[47] + Gu2[3];
Gu3[4] = + Q11[12]*Gu1[0] + Q11[13]*Gu1[4] + Q11[14]*Gu1[8] + Q11[15]*Gu1[12] + Q11[16]*Gu1[16] + Q11[17]*Gu1[20] + Q11[18]*Gu1[24] + Q11[19]*Gu1[28] + Q11[20]*Gu1[32] + Q11[21]*Gu1[36] + Q11[22]*Gu1[40] + Q11[23]*Gu1[44] + Gu2[4];
Gu3[5] = + Q11[12]*Gu1[1] + Q11[13]*Gu1[5] + Q11[14]*Gu1[9] + Q11[15]*Gu1[13] + Q11[16]*Gu1[17] + Q11[17]*Gu1[21] + Q11[18]*Gu1[25] + Q11[19]*Gu1[29] + Q11[20]*Gu1[33] + Q11[21]*Gu1[37] + Q11[22]*Gu1[41] + Q11[23]*Gu1[45] + Gu2[5];
Gu3[6] = + Q11[12]*Gu1[2] + Q11[13]*Gu1[6] + Q11[14]*Gu1[10] + Q11[15]*Gu1[14] + Q11[16]*Gu1[18] + Q11[17]*Gu1[22] + Q11[18]*Gu1[26] + Q11[19]*Gu1[30] + Q11[20]*Gu1[34] + Q11[21]*Gu1[38] + Q11[22]*Gu1[42] + Q11[23]*Gu1[46] + Gu2[6];
Gu3[7] = + Q11[12]*Gu1[3] + Q11[13]*Gu1[7] + Q11[14]*Gu1[11] + Q11[15]*Gu1[15] + Q11[16]*Gu1[19] + Q11[17]*Gu1[23] + Q11[18]*Gu1[27] + Q11[19]*Gu1[31] + Q11[20]*Gu1[35] + Q11[21]*Gu1[39] + Q11[22]*Gu1[43] + Q11[23]*Gu1[47] + Gu2[7];
Gu3[8] = + Q11[24]*Gu1[0] + Q11[25]*Gu1[4] + Q11[26]*Gu1[8] + Q11[27]*Gu1[12] + Q11[28]*Gu1[16] + Q11[29]*Gu1[20] + Q11[30]*Gu1[24] + Q11[31]*Gu1[28] + Q11[32]*Gu1[32] + Q11[33]*Gu1[36] + Q11[34]*Gu1[40] + Q11[35]*Gu1[44] + Gu2[8];
Gu3[9] = + Q11[24]*Gu1[1] + Q11[25]*Gu1[5] + Q11[26]*Gu1[9] + Q11[27]*Gu1[13] + Q11[28]*Gu1[17] + Q11[29]*Gu1[21] + Q11[30]*Gu1[25] + Q11[31]*Gu1[29] + Q11[32]*Gu1[33] + Q11[33]*Gu1[37] + Q11[34]*Gu1[41] + Q11[35]*Gu1[45] + Gu2[9];
Gu3[10] = + Q11[24]*Gu1[2] + Q11[25]*Gu1[6] + Q11[26]*Gu1[10] + Q11[27]*Gu1[14] + Q11[28]*Gu1[18] + Q11[29]*Gu1[22] + Q11[30]*Gu1[26] + Q11[31]*Gu1[30] + Q11[32]*Gu1[34] + Q11[33]*Gu1[38] + Q11[34]*Gu1[42] + Q11[35]*Gu1[46] + Gu2[10];
Gu3[11] = + Q11[24]*Gu1[3] + Q11[25]*Gu1[7] + Q11[26]*Gu1[11] + Q11[27]*Gu1[15] + Q11[28]*Gu1[19] + Q11[29]*Gu1[23] + Q11[30]*Gu1[27] + Q11[31]*Gu1[31] + Q11[32]*Gu1[35] + Q11[33]*Gu1[39] + Q11[34]*Gu1[43] + Q11[35]*Gu1[47] + Gu2[11];
Gu3[12] = + Q11[36]*Gu1[0] + Q11[37]*Gu1[4] + Q11[38]*Gu1[8] + Q11[39]*Gu1[12] + Q11[40]*Gu1[16] + Q11[41]*Gu1[20] + Q11[42]*Gu1[24] + Q11[43]*Gu1[28] + Q11[44]*Gu1[32] + Q11[45]*Gu1[36] + Q11[46]*Gu1[40] + Q11[47]*Gu1[44] + Gu2[12];
Gu3[13] = + Q11[36]*Gu1[1] + Q11[37]*Gu1[5] + Q11[38]*Gu1[9] + Q11[39]*Gu1[13] + Q11[40]*Gu1[17] + Q11[41]*Gu1[21] + Q11[42]*Gu1[25] + Q11[43]*Gu1[29] + Q11[44]*Gu1[33] + Q11[45]*Gu1[37] + Q11[46]*Gu1[41] + Q11[47]*Gu1[45] + Gu2[13];
Gu3[14] = + Q11[36]*Gu1[2] + Q11[37]*Gu1[6] + Q11[38]*Gu1[10] + Q11[39]*Gu1[14] + Q11[40]*Gu1[18] + Q11[41]*Gu1[22] + Q11[42]*Gu1[26] + Q11[43]*Gu1[30] + Q11[44]*Gu1[34] + Q11[45]*Gu1[38] + Q11[46]*Gu1[42] + Q11[47]*Gu1[46] + Gu2[14];
Gu3[15] = + Q11[36]*Gu1[3] + Q11[37]*Gu1[7] + Q11[38]*Gu1[11] + Q11[39]*Gu1[15] + Q11[40]*Gu1[19] + Q11[41]*Gu1[23] + Q11[42]*Gu1[27] + Q11[43]*Gu1[31] + Q11[44]*Gu1[35] + Q11[45]*Gu1[39] + Q11[46]*Gu1[43] + Q11[47]*Gu1[47] + Gu2[15];
Gu3[16] = + Q11[48]*Gu1[0] + Q11[49]*Gu1[4] + Q11[50]*Gu1[8] + Q11[51]*Gu1[12] + Q11[52]*Gu1[16] + Q11[53]*Gu1[20] + Q11[54]*Gu1[24] + Q11[55]*Gu1[28] + Q11[56]*Gu1[32] + Q11[57]*Gu1[36] + Q11[58]*Gu1[40] + Q11[59]*Gu1[44] + Gu2[16];
Gu3[17] = + Q11[48]*Gu1[1] + Q11[49]*Gu1[5] + Q11[50]*Gu1[9] + Q11[51]*Gu1[13] + Q11[52]*Gu1[17] + Q11[53]*Gu1[21] + Q11[54]*Gu1[25] + Q11[55]*Gu1[29] + Q11[56]*Gu1[33] + Q11[57]*Gu1[37] + Q11[58]*Gu1[41] + Q11[59]*Gu1[45] + Gu2[17];
Gu3[18] = + Q11[48]*Gu1[2] + Q11[49]*Gu1[6] + Q11[50]*Gu1[10] + Q11[51]*Gu1[14] + Q11[52]*Gu1[18] + Q11[53]*Gu1[22] + Q11[54]*Gu1[26] + Q11[55]*Gu1[30] + Q11[56]*Gu1[34] + Q11[57]*Gu1[38] + Q11[58]*Gu1[42] + Q11[59]*Gu1[46] + Gu2[18];
Gu3[19] = + Q11[48]*Gu1[3] + Q11[49]*Gu1[7] + Q11[50]*Gu1[11] + Q11[51]*Gu1[15] + Q11[52]*Gu1[19] + Q11[53]*Gu1[23] + Q11[54]*Gu1[27] + Q11[55]*Gu1[31] + Q11[56]*Gu1[35] + Q11[57]*Gu1[39] + Q11[58]*Gu1[43] + Q11[59]*Gu1[47] + Gu2[19];
Gu3[20] = + Q11[60]*Gu1[0] + Q11[61]*Gu1[4] + Q11[62]*Gu1[8] + Q11[63]*Gu1[12] + Q11[64]*Gu1[16] + Q11[65]*Gu1[20] + Q11[66]*Gu1[24] + Q11[67]*Gu1[28] + Q11[68]*Gu1[32] + Q11[69]*Gu1[36] + Q11[70]*Gu1[40] + Q11[71]*Gu1[44] + Gu2[20];
Gu3[21] = + Q11[60]*Gu1[1] + Q11[61]*Gu1[5] + Q11[62]*Gu1[9] + Q11[63]*Gu1[13] + Q11[64]*Gu1[17] + Q11[65]*Gu1[21] + Q11[66]*Gu1[25] + Q11[67]*Gu1[29] + Q11[68]*Gu1[33] + Q11[69]*Gu1[37] + Q11[70]*Gu1[41] + Q11[71]*Gu1[45] + Gu2[21];
Gu3[22] = + Q11[60]*Gu1[2] + Q11[61]*Gu1[6] + Q11[62]*Gu1[10] + Q11[63]*Gu1[14] + Q11[64]*Gu1[18] + Q11[65]*Gu1[22] + Q11[66]*Gu1[26] + Q11[67]*Gu1[30] + Q11[68]*Gu1[34] + Q11[69]*Gu1[38] + Q11[70]*Gu1[42] + Q11[71]*Gu1[46] + Gu2[22];
Gu3[23] = + Q11[60]*Gu1[3] + Q11[61]*Gu1[7] + Q11[62]*Gu1[11] + Q11[63]*Gu1[15] + Q11[64]*Gu1[19] + Q11[65]*Gu1[23] + Q11[66]*Gu1[27] + Q11[67]*Gu1[31] + Q11[68]*Gu1[35] + Q11[69]*Gu1[39] + Q11[70]*Gu1[43] + Q11[71]*Gu1[47] + Gu2[23];
Gu3[24] = + Q11[72]*Gu1[0] + Q11[73]*Gu1[4] + Q11[74]*Gu1[8] + Q11[75]*Gu1[12] + Q11[76]*Gu1[16] + Q11[77]*Gu1[20] + Q11[78]*Gu1[24] + Q11[79]*Gu1[28] + Q11[80]*Gu1[32] + Q11[81]*Gu1[36] + Q11[82]*Gu1[40] + Q11[83]*Gu1[44] + Gu2[24];
Gu3[25] = + Q11[72]*Gu1[1] + Q11[73]*Gu1[5] + Q11[74]*Gu1[9] + Q11[75]*Gu1[13] + Q11[76]*Gu1[17] + Q11[77]*Gu1[21] + Q11[78]*Gu1[25] + Q11[79]*Gu1[29] + Q11[80]*Gu1[33] + Q11[81]*Gu1[37] + Q11[82]*Gu1[41] + Q11[83]*Gu1[45] + Gu2[25];
Gu3[26] = + Q11[72]*Gu1[2] + Q11[73]*Gu1[6] + Q11[74]*Gu1[10] + Q11[75]*Gu1[14] + Q11[76]*Gu1[18] + Q11[77]*Gu1[22] + Q11[78]*Gu1[26] + Q11[79]*Gu1[30] + Q11[80]*Gu1[34] + Q11[81]*Gu1[38] + Q11[82]*Gu1[42] + Q11[83]*Gu1[46] + Gu2[26];
Gu3[27] = + Q11[72]*Gu1[3] + Q11[73]*Gu1[7] + Q11[74]*Gu1[11] + Q11[75]*Gu1[15] + Q11[76]*Gu1[19] + Q11[77]*Gu1[23] + Q11[78]*Gu1[27] + Q11[79]*Gu1[31] + Q11[80]*Gu1[35] + Q11[81]*Gu1[39] + Q11[82]*Gu1[43] + Q11[83]*Gu1[47] + Gu2[27];
Gu3[28] = + Q11[84]*Gu1[0] + Q11[85]*Gu1[4] + Q11[86]*Gu1[8] + Q11[87]*Gu1[12] + Q11[88]*Gu1[16] + Q11[89]*Gu1[20] + Q11[90]*Gu1[24] + Q11[91]*Gu1[28] + Q11[92]*Gu1[32] + Q11[93]*Gu1[36] + Q11[94]*Gu1[40] + Q11[95]*Gu1[44] + Gu2[28];
Gu3[29] = + Q11[84]*Gu1[1] + Q11[85]*Gu1[5] + Q11[86]*Gu1[9] + Q11[87]*Gu1[13] + Q11[88]*Gu1[17] + Q11[89]*Gu1[21] + Q11[90]*Gu1[25] + Q11[91]*Gu1[29] + Q11[92]*Gu1[33] + Q11[93]*Gu1[37] + Q11[94]*Gu1[41] + Q11[95]*Gu1[45] + Gu2[29];
Gu3[30] = + Q11[84]*Gu1[2] + Q11[85]*Gu1[6] + Q11[86]*Gu1[10] + Q11[87]*Gu1[14] + Q11[88]*Gu1[18] + Q11[89]*Gu1[22] + Q11[90]*Gu1[26] + Q11[91]*Gu1[30] + Q11[92]*Gu1[34] + Q11[93]*Gu1[38] + Q11[94]*Gu1[42] + Q11[95]*Gu1[46] + Gu2[30];
Gu3[31] = + Q11[84]*Gu1[3] + Q11[85]*Gu1[7] + Q11[86]*Gu1[11] + Q11[87]*Gu1[15] + Q11[88]*Gu1[19] + Q11[89]*Gu1[23] + Q11[90]*Gu1[27] + Q11[91]*Gu1[31] + Q11[92]*Gu1[35] + Q11[93]*Gu1[39] + Q11[94]*Gu1[43] + Q11[95]*Gu1[47] + Gu2[31];
Gu3[32] = + Q11[96]*Gu1[0] + Q11[97]*Gu1[4] + Q11[98]*Gu1[8] + Q11[99]*Gu1[12] + Q11[100]*Gu1[16] + Q11[101]*Gu1[20] + Q11[102]*Gu1[24] + Q11[103]*Gu1[28] + Q11[104]*Gu1[32] + Q11[105]*Gu1[36] + Q11[106]*Gu1[40] + Q11[107]*Gu1[44] + Gu2[32];
Gu3[33] = + Q11[96]*Gu1[1] + Q11[97]*Gu1[5] + Q11[98]*Gu1[9] + Q11[99]*Gu1[13] + Q11[100]*Gu1[17] + Q11[101]*Gu1[21] + Q11[102]*Gu1[25] + Q11[103]*Gu1[29] + Q11[104]*Gu1[33] + Q11[105]*Gu1[37] + Q11[106]*Gu1[41] + Q11[107]*Gu1[45] + Gu2[33];
Gu3[34] = + Q11[96]*Gu1[2] + Q11[97]*Gu1[6] + Q11[98]*Gu1[10] + Q11[99]*Gu1[14] + Q11[100]*Gu1[18] + Q11[101]*Gu1[22] + Q11[102]*Gu1[26] + Q11[103]*Gu1[30] + Q11[104]*Gu1[34] + Q11[105]*Gu1[38] + Q11[106]*Gu1[42] + Q11[107]*Gu1[46] + Gu2[34];
Gu3[35] = + Q11[96]*Gu1[3] + Q11[97]*Gu1[7] + Q11[98]*Gu1[11] + Q11[99]*Gu1[15] + Q11[100]*Gu1[19] + Q11[101]*Gu1[23] + Q11[102]*Gu1[27] + Q11[103]*Gu1[31] + Q11[104]*Gu1[35] + Q11[105]*Gu1[39] + Q11[106]*Gu1[43] + Q11[107]*Gu1[47] + Gu2[35];
Gu3[36] = + Q11[108]*Gu1[0] + Q11[109]*Gu1[4] + Q11[110]*Gu1[8] + Q11[111]*Gu1[12] + Q11[112]*Gu1[16] + Q11[113]*Gu1[20] + Q11[114]*Gu1[24] + Q11[115]*Gu1[28] + Q11[116]*Gu1[32] + Q11[117]*Gu1[36] + Q11[118]*Gu1[40] + Q11[119]*Gu1[44] + Gu2[36];
Gu3[37] = + Q11[108]*Gu1[1] + Q11[109]*Gu1[5] + Q11[110]*Gu1[9] + Q11[111]*Gu1[13] + Q11[112]*Gu1[17] + Q11[113]*Gu1[21] + Q11[114]*Gu1[25] + Q11[115]*Gu1[29] + Q11[116]*Gu1[33] + Q11[117]*Gu1[37] + Q11[118]*Gu1[41] + Q11[119]*Gu1[45] + Gu2[37];
Gu3[38] = + Q11[108]*Gu1[2] + Q11[109]*Gu1[6] + Q11[110]*Gu1[10] + Q11[111]*Gu1[14] + Q11[112]*Gu1[18] + Q11[113]*Gu1[22] + Q11[114]*Gu1[26] + Q11[115]*Gu1[30] + Q11[116]*Gu1[34] + Q11[117]*Gu1[38] + Q11[118]*Gu1[42] + Q11[119]*Gu1[46] + Gu2[38];
Gu3[39] = + Q11[108]*Gu1[3] + Q11[109]*Gu1[7] + Q11[110]*Gu1[11] + Q11[111]*Gu1[15] + Q11[112]*Gu1[19] + Q11[113]*Gu1[23] + Q11[114]*Gu1[27] + Q11[115]*Gu1[31] + Q11[116]*Gu1[35] + Q11[117]*Gu1[39] + Q11[118]*Gu1[43] + Q11[119]*Gu1[47] + Gu2[39];
Gu3[40] = + Q11[120]*Gu1[0] + Q11[121]*Gu1[4] + Q11[122]*Gu1[8] + Q11[123]*Gu1[12] + Q11[124]*Gu1[16] + Q11[125]*Gu1[20] + Q11[126]*Gu1[24] + Q11[127]*Gu1[28] + Q11[128]*Gu1[32] + Q11[129]*Gu1[36] + Q11[130]*Gu1[40] + Q11[131]*Gu1[44] + Gu2[40];
Gu3[41] = + Q11[120]*Gu1[1] + Q11[121]*Gu1[5] + Q11[122]*Gu1[9] + Q11[123]*Gu1[13] + Q11[124]*Gu1[17] + Q11[125]*Gu1[21] + Q11[126]*Gu1[25] + Q11[127]*Gu1[29] + Q11[128]*Gu1[33] + Q11[129]*Gu1[37] + Q11[130]*Gu1[41] + Q11[131]*Gu1[45] + Gu2[41];
Gu3[42] = + Q11[120]*Gu1[2] + Q11[121]*Gu1[6] + Q11[122]*Gu1[10] + Q11[123]*Gu1[14] + Q11[124]*Gu1[18] + Q11[125]*Gu1[22] + Q11[126]*Gu1[26] + Q11[127]*Gu1[30] + Q11[128]*Gu1[34] + Q11[129]*Gu1[38] + Q11[130]*Gu1[42] + Q11[131]*Gu1[46] + Gu2[42];
Gu3[43] = + Q11[120]*Gu1[3] + Q11[121]*Gu1[7] + Q11[122]*Gu1[11] + Q11[123]*Gu1[15] + Q11[124]*Gu1[19] + Q11[125]*Gu1[23] + Q11[126]*Gu1[27] + Q11[127]*Gu1[31] + Q11[128]*Gu1[35] + Q11[129]*Gu1[39] + Q11[130]*Gu1[43] + Q11[131]*Gu1[47] + Gu2[43];
Gu3[44] = + Q11[132]*Gu1[0] + Q11[133]*Gu1[4] + Q11[134]*Gu1[8] + Q11[135]*Gu1[12] + Q11[136]*Gu1[16] + Q11[137]*Gu1[20] + Q11[138]*Gu1[24] + Q11[139]*Gu1[28] + Q11[140]*Gu1[32] + Q11[141]*Gu1[36] + Q11[142]*Gu1[40] + Q11[143]*Gu1[44] + Gu2[44];
Gu3[45] = + Q11[132]*Gu1[1] + Q11[133]*Gu1[5] + Q11[134]*Gu1[9] + Q11[135]*Gu1[13] + Q11[136]*Gu1[17] + Q11[137]*Gu1[21] + Q11[138]*Gu1[25] + Q11[139]*Gu1[29] + Q11[140]*Gu1[33] + Q11[141]*Gu1[37] + Q11[142]*Gu1[41] + Q11[143]*Gu1[45] + Gu2[45];
Gu3[46] = + Q11[132]*Gu1[2] + Q11[133]*Gu1[6] + Q11[134]*Gu1[10] + Q11[135]*Gu1[14] + Q11[136]*Gu1[18] + Q11[137]*Gu1[22] + Q11[138]*Gu1[26] + Q11[139]*Gu1[30] + Q11[140]*Gu1[34] + Q11[141]*Gu1[38] + Q11[142]*Gu1[42] + Q11[143]*Gu1[46] + Gu2[46];
Gu3[47] = + Q11[132]*Gu1[3] + Q11[133]*Gu1[7] + Q11[134]*Gu1[11] + Q11[135]*Gu1[15] + Q11[136]*Gu1[19] + Q11[137]*Gu1[23] + Q11[138]*Gu1[27] + Q11[139]*Gu1[31] + Q11[140]*Gu1[35] + Q11[141]*Gu1[39] + Q11[142]*Gu1[43] + Q11[143]*Gu1[47] + Gu2[47];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[12]*w11[1] + Gx1[24]*w11[2] + Gx1[36]*w11[3] + Gx1[48]*w11[4] + Gx1[60]*w11[5] + Gx1[72]*w11[6] + Gx1[84]*w11[7] + Gx1[96]*w11[8] + Gx1[108]*w11[9] + Gx1[120]*w11[10] + Gx1[132]*w11[11] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[13]*w11[1] + Gx1[25]*w11[2] + Gx1[37]*w11[3] + Gx1[49]*w11[4] + Gx1[61]*w11[5] + Gx1[73]*w11[6] + Gx1[85]*w11[7] + Gx1[97]*w11[8] + Gx1[109]*w11[9] + Gx1[121]*w11[10] + Gx1[133]*w11[11] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[14]*w11[1] + Gx1[26]*w11[2] + Gx1[38]*w11[3] + Gx1[50]*w11[4] + Gx1[62]*w11[5] + Gx1[74]*w11[6] + Gx1[86]*w11[7] + Gx1[98]*w11[8] + Gx1[110]*w11[9] + Gx1[122]*w11[10] + Gx1[134]*w11[11] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[15]*w11[1] + Gx1[27]*w11[2] + Gx1[39]*w11[3] + Gx1[51]*w11[4] + Gx1[63]*w11[5] + Gx1[75]*w11[6] + Gx1[87]*w11[7] + Gx1[99]*w11[8] + Gx1[111]*w11[9] + Gx1[123]*w11[10] + Gx1[135]*w11[11] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[16]*w11[1] + Gx1[28]*w11[2] + Gx1[40]*w11[3] + Gx1[52]*w11[4] + Gx1[64]*w11[5] + Gx1[76]*w11[6] + Gx1[88]*w11[7] + Gx1[100]*w11[8] + Gx1[112]*w11[9] + Gx1[124]*w11[10] + Gx1[136]*w11[11] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[17]*w11[1] + Gx1[29]*w11[2] + Gx1[41]*w11[3] + Gx1[53]*w11[4] + Gx1[65]*w11[5] + Gx1[77]*w11[6] + Gx1[89]*w11[7] + Gx1[101]*w11[8] + Gx1[113]*w11[9] + Gx1[125]*w11[10] + Gx1[137]*w11[11] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[18]*w11[1] + Gx1[30]*w11[2] + Gx1[42]*w11[3] + Gx1[54]*w11[4] + Gx1[66]*w11[5] + Gx1[78]*w11[6] + Gx1[90]*w11[7] + Gx1[102]*w11[8] + Gx1[114]*w11[9] + Gx1[126]*w11[10] + Gx1[138]*w11[11] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[19]*w11[1] + Gx1[31]*w11[2] + Gx1[43]*w11[3] + Gx1[55]*w11[4] + Gx1[67]*w11[5] + Gx1[79]*w11[6] + Gx1[91]*w11[7] + Gx1[103]*w11[8] + Gx1[115]*w11[9] + Gx1[127]*w11[10] + Gx1[139]*w11[11] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[20]*w11[1] + Gx1[32]*w11[2] + Gx1[44]*w11[3] + Gx1[56]*w11[4] + Gx1[68]*w11[5] + Gx1[80]*w11[6] + Gx1[92]*w11[7] + Gx1[104]*w11[8] + Gx1[116]*w11[9] + Gx1[128]*w11[10] + Gx1[140]*w11[11] + w12[8];
w13[9] = + Gx1[9]*w11[0] + Gx1[21]*w11[1] + Gx1[33]*w11[2] + Gx1[45]*w11[3] + Gx1[57]*w11[4] + Gx1[69]*w11[5] + Gx1[81]*w11[6] + Gx1[93]*w11[7] + Gx1[105]*w11[8] + Gx1[117]*w11[9] + Gx1[129]*w11[10] + Gx1[141]*w11[11] + w12[9];
w13[10] = + Gx1[10]*w11[0] + Gx1[22]*w11[1] + Gx1[34]*w11[2] + Gx1[46]*w11[3] + Gx1[58]*w11[4] + Gx1[70]*w11[5] + Gx1[82]*w11[6] + Gx1[94]*w11[7] + Gx1[106]*w11[8] + Gx1[118]*w11[9] + Gx1[130]*w11[10] + Gx1[142]*w11[11] + w12[10];
w13[11] = + Gx1[11]*w11[0] + Gx1[23]*w11[1] + Gx1[35]*w11[2] + Gx1[47]*w11[3] + Gx1[59]*w11[4] + Gx1[71]*w11[5] + Gx1[83]*w11[6] + Gx1[95]*w11[7] + Gx1[107]*w11[8] + Gx1[119]*w11[9] + Gx1[131]*w11[10] + Gx1[143]*w11[11] + w12[11];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[4]*w11[1] + Gu1[8]*w11[2] + Gu1[12]*w11[3] + Gu1[16]*w11[4] + Gu1[20]*w11[5] + Gu1[24]*w11[6] + Gu1[28]*w11[7] + Gu1[32]*w11[8] + Gu1[36]*w11[9] + Gu1[40]*w11[10] + Gu1[44]*w11[11];
U1[1] += + Gu1[1]*w11[0] + Gu1[5]*w11[1] + Gu1[9]*w11[2] + Gu1[13]*w11[3] + Gu1[17]*w11[4] + Gu1[21]*w11[5] + Gu1[25]*w11[6] + Gu1[29]*w11[7] + Gu1[33]*w11[8] + Gu1[37]*w11[9] + Gu1[41]*w11[10] + Gu1[45]*w11[11];
U1[2] += + Gu1[2]*w11[0] + Gu1[6]*w11[1] + Gu1[10]*w11[2] + Gu1[14]*w11[3] + Gu1[18]*w11[4] + Gu1[22]*w11[5] + Gu1[26]*w11[6] + Gu1[30]*w11[7] + Gu1[34]*w11[8] + Gu1[38]*w11[9] + Gu1[42]*w11[10] + Gu1[46]*w11[11];
U1[3] += + Gu1[3]*w11[0] + Gu1[7]*w11[1] + Gu1[11]*w11[2] + Gu1[15]*w11[3] + Gu1[19]*w11[4] + Gu1[23]*w11[5] + Gu1[27]*w11[6] + Gu1[31]*w11[7] + Gu1[35]*w11[8] + Gu1[39]*w11[9] + Gu1[43]*w11[10] + Gu1[47]*w11[11];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + Q11[9]*w11[9] + Q11[10]*w11[10] + Q11[11]*w11[11] + w12[0];
w13[1] = + Q11[12]*w11[0] + Q11[13]*w11[1] + Q11[14]*w11[2] + Q11[15]*w11[3] + Q11[16]*w11[4] + Q11[17]*w11[5] + Q11[18]*w11[6] + Q11[19]*w11[7] + Q11[20]*w11[8] + Q11[21]*w11[9] + Q11[22]*w11[10] + Q11[23]*w11[11] + w12[1];
w13[2] = + Q11[24]*w11[0] + Q11[25]*w11[1] + Q11[26]*w11[2] + Q11[27]*w11[3] + Q11[28]*w11[4] + Q11[29]*w11[5] + Q11[30]*w11[6] + Q11[31]*w11[7] + Q11[32]*w11[8] + Q11[33]*w11[9] + Q11[34]*w11[10] + Q11[35]*w11[11] + w12[2];
w13[3] = + Q11[36]*w11[0] + Q11[37]*w11[1] + Q11[38]*w11[2] + Q11[39]*w11[3] + Q11[40]*w11[4] + Q11[41]*w11[5] + Q11[42]*w11[6] + Q11[43]*w11[7] + Q11[44]*w11[8] + Q11[45]*w11[9] + Q11[46]*w11[10] + Q11[47]*w11[11] + w12[3];
w13[4] = + Q11[48]*w11[0] + Q11[49]*w11[1] + Q11[50]*w11[2] + Q11[51]*w11[3] + Q11[52]*w11[4] + Q11[53]*w11[5] + Q11[54]*w11[6] + Q11[55]*w11[7] + Q11[56]*w11[8] + Q11[57]*w11[9] + Q11[58]*w11[10] + Q11[59]*w11[11] + w12[4];
w13[5] = + Q11[60]*w11[0] + Q11[61]*w11[1] + Q11[62]*w11[2] + Q11[63]*w11[3] + Q11[64]*w11[4] + Q11[65]*w11[5] + Q11[66]*w11[6] + Q11[67]*w11[7] + Q11[68]*w11[8] + Q11[69]*w11[9] + Q11[70]*w11[10] + Q11[71]*w11[11] + w12[5];
w13[6] = + Q11[72]*w11[0] + Q11[73]*w11[1] + Q11[74]*w11[2] + Q11[75]*w11[3] + Q11[76]*w11[4] + Q11[77]*w11[5] + Q11[78]*w11[6] + Q11[79]*w11[7] + Q11[80]*w11[8] + Q11[81]*w11[9] + Q11[82]*w11[10] + Q11[83]*w11[11] + w12[6];
w13[7] = + Q11[84]*w11[0] + Q11[85]*w11[1] + Q11[86]*w11[2] + Q11[87]*w11[3] + Q11[88]*w11[4] + Q11[89]*w11[5] + Q11[90]*w11[6] + Q11[91]*w11[7] + Q11[92]*w11[8] + Q11[93]*w11[9] + Q11[94]*w11[10] + Q11[95]*w11[11] + w12[7];
w13[8] = + Q11[96]*w11[0] + Q11[97]*w11[1] + Q11[98]*w11[2] + Q11[99]*w11[3] + Q11[100]*w11[4] + Q11[101]*w11[5] + Q11[102]*w11[6] + Q11[103]*w11[7] + Q11[104]*w11[8] + Q11[105]*w11[9] + Q11[106]*w11[10] + Q11[107]*w11[11] + w12[8];
w13[9] = + Q11[108]*w11[0] + Q11[109]*w11[1] + Q11[110]*w11[2] + Q11[111]*w11[3] + Q11[112]*w11[4] + Q11[113]*w11[5] + Q11[114]*w11[6] + Q11[115]*w11[7] + Q11[116]*w11[8] + Q11[117]*w11[9] + Q11[118]*w11[10] + Q11[119]*w11[11] + w12[9];
w13[10] = + Q11[120]*w11[0] + Q11[121]*w11[1] + Q11[122]*w11[2] + Q11[123]*w11[3] + Q11[124]*w11[4] + Q11[125]*w11[5] + Q11[126]*w11[6] + Q11[127]*w11[7] + Q11[128]*w11[8] + Q11[129]*w11[9] + Q11[130]*w11[10] + Q11[131]*w11[11] + w12[10];
w13[11] = + Q11[132]*w11[0] + Q11[133]*w11[1] + Q11[134]*w11[2] + Q11[135]*w11[3] + Q11[136]*w11[4] + Q11[137]*w11[5] + Q11[138]*w11[6] + Q11[139]*w11[7] + Q11[140]*w11[8] + Q11[141]*w11[9] + Q11[142]*w11[10] + Q11[143]*w11[11] + w12[11];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11];
w12[1] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5] + Gx1[18]*w11[6] + Gx1[19]*w11[7] + Gx1[20]*w11[8] + Gx1[21]*w11[9] + Gx1[22]*w11[10] + Gx1[23]*w11[11];
w12[2] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5] + Gx1[30]*w11[6] + Gx1[31]*w11[7] + Gx1[32]*w11[8] + Gx1[33]*w11[9] + Gx1[34]*w11[10] + Gx1[35]*w11[11];
w12[3] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8] + Gx1[45]*w11[9] + Gx1[46]*w11[10] + Gx1[47]*w11[11];
w12[4] += + Gx1[48]*w11[0] + Gx1[49]*w11[1] + Gx1[50]*w11[2] + Gx1[51]*w11[3] + Gx1[52]*w11[4] + Gx1[53]*w11[5] + Gx1[54]*w11[6] + Gx1[55]*w11[7] + Gx1[56]*w11[8] + Gx1[57]*w11[9] + Gx1[58]*w11[10] + Gx1[59]*w11[11];
w12[5] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9] + Gx1[70]*w11[10] + Gx1[71]*w11[11];
w12[6] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8] + Gx1[81]*w11[9] + Gx1[82]*w11[10] + Gx1[83]*w11[11];
w12[7] += + Gx1[84]*w11[0] + Gx1[85]*w11[1] + Gx1[86]*w11[2] + Gx1[87]*w11[3] + Gx1[88]*w11[4] + Gx1[89]*w11[5] + Gx1[90]*w11[6] + Gx1[91]*w11[7] + Gx1[92]*w11[8] + Gx1[93]*w11[9] + Gx1[94]*w11[10] + Gx1[95]*w11[11];
w12[8] += + Gx1[96]*w11[0] + Gx1[97]*w11[1] + Gx1[98]*w11[2] + Gx1[99]*w11[3] + Gx1[100]*w11[4] + Gx1[101]*w11[5] + Gx1[102]*w11[6] + Gx1[103]*w11[7] + Gx1[104]*w11[8] + Gx1[105]*w11[9] + Gx1[106]*w11[10] + Gx1[107]*w11[11];
w12[9] += + Gx1[108]*w11[0] + Gx1[109]*w11[1] + Gx1[110]*w11[2] + Gx1[111]*w11[3] + Gx1[112]*w11[4] + Gx1[113]*w11[5] + Gx1[114]*w11[6] + Gx1[115]*w11[7] + Gx1[116]*w11[8] + Gx1[117]*w11[9] + Gx1[118]*w11[10] + Gx1[119]*w11[11];
w12[10] += + Gx1[120]*w11[0] + Gx1[121]*w11[1] + Gx1[122]*w11[2] + Gx1[123]*w11[3] + Gx1[124]*w11[4] + Gx1[125]*w11[5] + Gx1[126]*w11[6] + Gx1[127]*w11[7] + Gx1[128]*w11[8] + Gx1[129]*w11[9] + Gx1[130]*w11[10] + Gx1[131]*w11[11];
w12[11] += + Gx1[132]*w11[0] + Gx1[133]*w11[1] + Gx1[134]*w11[2] + Gx1[135]*w11[3] + Gx1[136]*w11[4] + Gx1[137]*w11[5] + Gx1[138]*w11[6] + Gx1[139]*w11[7] + Gx1[140]*w11[8] + Gx1[141]*w11[9] + Gx1[142]*w11[10] + Gx1[143]*w11[11];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11];
w12[1] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5] + Gx1[18]*w11[6] + Gx1[19]*w11[7] + Gx1[20]*w11[8] + Gx1[21]*w11[9] + Gx1[22]*w11[10] + Gx1[23]*w11[11];
w12[2] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5] + Gx1[30]*w11[6] + Gx1[31]*w11[7] + Gx1[32]*w11[8] + Gx1[33]*w11[9] + Gx1[34]*w11[10] + Gx1[35]*w11[11];
w12[3] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8] + Gx1[45]*w11[9] + Gx1[46]*w11[10] + Gx1[47]*w11[11];
w12[4] += + Gx1[48]*w11[0] + Gx1[49]*w11[1] + Gx1[50]*w11[2] + Gx1[51]*w11[3] + Gx1[52]*w11[4] + Gx1[53]*w11[5] + Gx1[54]*w11[6] + Gx1[55]*w11[7] + Gx1[56]*w11[8] + Gx1[57]*w11[9] + Gx1[58]*w11[10] + Gx1[59]*w11[11];
w12[5] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9] + Gx1[70]*w11[10] + Gx1[71]*w11[11];
w12[6] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8] + Gx1[81]*w11[9] + Gx1[82]*w11[10] + Gx1[83]*w11[11];
w12[7] += + Gx1[84]*w11[0] + Gx1[85]*w11[1] + Gx1[86]*w11[2] + Gx1[87]*w11[3] + Gx1[88]*w11[4] + Gx1[89]*w11[5] + Gx1[90]*w11[6] + Gx1[91]*w11[7] + Gx1[92]*w11[8] + Gx1[93]*w11[9] + Gx1[94]*w11[10] + Gx1[95]*w11[11];
w12[8] += + Gx1[96]*w11[0] + Gx1[97]*w11[1] + Gx1[98]*w11[2] + Gx1[99]*w11[3] + Gx1[100]*w11[4] + Gx1[101]*w11[5] + Gx1[102]*w11[6] + Gx1[103]*w11[7] + Gx1[104]*w11[8] + Gx1[105]*w11[9] + Gx1[106]*w11[10] + Gx1[107]*w11[11];
w12[9] += + Gx1[108]*w11[0] + Gx1[109]*w11[1] + Gx1[110]*w11[2] + Gx1[111]*w11[3] + Gx1[112]*w11[4] + Gx1[113]*w11[5] + Gx1[114]*w11[6] + Gx1[115]*w11[7] + Gx1[116]*w11[8] + Gx1[117]*w11[9] + Gx1[118]*w11[10] + Gx1[119]*w11[11];
w12[10] += + Gx1[120]*w11[0] + Gx1[121]*w11[1] + Gx1[122]*w11[2] + Gx1[123]*w11[3] + Gx1[124]*w11[4] + Gx1[125]*w11[5] + Gx1[126]*w11[6] + Gx1[127]*w11[7] + Gx1[128]*w11[8] + Gx1[129]*w11[9] + Gx1[130]*w11[10] + Gx1[131]*w11[11];
w12[11] += + Gx1[132]*w11[0] + Gx1[133]*w11[1] + Gx1[134]*w11[2] + Gx1[135]*w11[3] + Gx1[136]*w11[4] + Gx1[137]*w11[5] + Gx1[138]*w11[6] + Gx1[139]*w11[7] + Gx1[140]*w11[8] + Gx1[141]*w11[9] + Gx1[142]*w11[10] + Gx1[143]*w11[11];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2] + Gu1[3]*U1[3];
w12[1] += + Gu1[4]*U1[0] + Gu1[5]*U1[1] + Gu1[6]*U1[2] + Gu1[7]*U1[3];
w12[2] += + Gu1[8]*U1[0] + Gu1[9]*U1[1] + Gu1[10]*U1[2] + Gu1[11]*U1[3];
w12[3] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2] + Gu1[15]*U1[3];
w12[4] += + Gu1[16]*U1[0] + Gu1[17]*U1[1] + Gu1[18]*U1[2] + Gu1[19]*U1[3];
w12[5] += + Gu1[20]*U1[0] + Gu1[21]*U1[1] + Gu1[22]*U1[2] + Gu1[23]*U1[3];
w12[6] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2] + Gu1[27]*U1[3];
w12[7] += + Gu1[28]*U1[0] + Gu1[29]*U1[1] + Gu1[30]*U1[2] + Gu1[31]*U1[3];
w12[8] += + Gu1[32]*U1[0] + Gu1[33]*U1[1] + Gu1[34]*U1[2] + Gu1[35]*U1[3];
w12[9] += + Gu1[36]*U1[0] + Gu1[37]*U1[1] + Gu1[38]*U1[2] + Gu1[39]*U1[3];
w12[10] += + Gu1[40]*U1[0] + Gu1[41]*U1[1] + Gu1[42]*U1[2] + Gu1[43]*U1[3];
w12[11] += + Gu1[44]*U1[0] + Gu1[45]*U1[1] + Gu1[46]*U1[2] + Gu1[47]*U1[3];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 160) + (iCol * 4)] = acadoWorkspace.H[(iCol * 160) + (iRow * 4)];
acadoWorkspace.H[(iRow * 160) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 160 + 40) + (iRow * 4)];
acadoWorkspace.H[(iRow * 160) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 160 + 80) + (iRow * 4)];
acadoWorkspace.H[(iRow * 160) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 160 + 120) + (iRow * 4)];
acadoWorkspace.H[(iRow * 160 + 40) + (iCol * 4)] = acadoWorkspace.H[(iCol * 160) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 160 + 40) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 160 + 40) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 160 + 40) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 160 + 80) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 160 + 40) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 160 + 120) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 160 + 80) + (iCol * 4)] = acadoWorkspace.H[(iCol * 160) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 160 + 80) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 160 + 40) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 160 + 80) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 160 + 80) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 160 + 80) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 160 + 120) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 160 + 120) + (iCol * 4)] = acadoWorkspace.H[(iCol * 160) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 160 + 120) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 160 + 40) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 160 + 120) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 160 + 80) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 160 + 120) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 160 + 120) + (iRow * 4 + 3)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12] + R2[13]*Dy1[13] + R2[14]*Dy1[14] + R2[15]*Dy1[15];
RDy1[1] = + R2[16]*Dy1[0] + R2[17]*Dy1[1] + R2[18]*Dy1[2] + R2[19]*Dy1[3] + R2[20]*Dy1[4] + R2[21]*Dy1[5] + R2[22]*Dy1[6] + R2[23]*Dy1[7] + R2[24]*Dy1[8] + R2[25]*Dy1[9] + R2[26]*Dy1[10] + R2[27]*Dy1[11] + R2[28]*Dy1[12] + R2[29]*Dy1[13] + R2[30]*Dy1[14] + R2[31]*Dy1[15];
RDy1[2] = + R2[32]*Dy1[0] + R2[33]*Dy1[1] + R2[34]*Dy1[2] + R2[35]*Dy1[3] + R2[36]*Dy1[4] + R2[37]*Dy1[5] + R2[38]*Dy1[6] + R2[39]*Dy1[7] + R2[40]*Dy1[8] + R2[41]*Dy1[9] + R2[42]*Dy1[10] + R2[43]*Dy1[11] + R2[44]*Dy1[12] + R2[45]*Dy1[13] + R2[46]*Dy1[14] + R2[47]*Dy1[15];
RDy1[3] = + R2[48]*Dy1[0] + R2[49]*Dy1[1] + R2[50]*Dy1[2] + R2[51]*Dy1[3] + R2[52]*Dy1[4] + R2[53]*Dy1[5] + R2[54]*Dy1[6] + R2[55]*Dy1[7] + R2[56]*Dy1[8] + R2[57]*Dy1[9] + R2[58]*Dy1[10] + R2[59]*Dy1[11] + R2[60]*Dy1[12] + R2[61]*Dy1[13] + R2[62]*Dy1[14] + R2[63]*Dy1[15];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12] + Q2[13]*Dy1[13] + Q2[14]*Dy1[14] + Q2[15]*Dy1[15];
QDy1[1] = + Q2[16]*Dy1[0] + Q2[17]*Dy1[1] + Q2[18]*Dy1[2] + Q2[19]*Dy1[3] + Q2[20]*Dy1[4] + Q2[21]*Dy1[5] + Q2[22]*Dy1[6] + Q2[23]*Dy1[7] + Q2[24]*Dy1[8] + Q2[25]*Dy1[9] + Q2[26]*Dy1[10] + Q2[27]*Dy1[11] + Q2[28]*Dy1[12] + Q2[29]*Dy1[13] + Q2[30]*Dy1[14] + Q2[31]*Dy1[15];
QDy1[2] = + Q2[32]*Dy1[0] + Q2[33]*Dy1[1] + Q2[34]*Dy1[2] + Q2[35]*Dy1[3] + Q2[36]*Dy1[4] + Q2[37]*Dy1[5] + Q2[38]*Dy1[6] + Q2[39]*Dy1[7] + Q2[40]*Dy1[8] + Q2[41]*Dy1[9] + Q2[42]*Dy1[10] + Q2[43]*Dy1[11] + Q2[44]*Dy1[12] + Q2[45]*Dy1[13] + Q2[46]*Dy1[14] + Q2[47]*Dy1[15];
QDy1[3] = + Q2[48]*Dy1[0] + Q2[49]*Dy1[1] + Q2[50]*Dy1[2] + Q2[51]*Dy1[3] + Q2[52]*Dy1[4] + Q2[53]*Dy1[5] + Q2[54]*Dy1[6] + Q2[55]*Dy1[7] + Q2[56]*Dy1[8] + Q2[57]*Dy1[9] + Q2[58]*Dy1[10] + Q2[59]*Dy1[11] + Q2[60]*Dy1[12] + Q2[61]*Dy1[13] + Q2[62]*Dy1[14] + Q2[63]*Dy1[15];
QDy1[4] = + Q2[64]*Dy1[0] + Q2[65]*Dy1[1] + Q2[66]*Dy1[2] + Q2[67]*Dy1[3] + Q2[68]*Dy1[4] + Q2[69]*Dy1[5] + Q2[70]*Dy1[6] + Q2[71]*Dy1[7] + Q2[72]*Dy1[8] + Q2[73]*Dy1[9] + Q2[74]*Dy1[10] + Q2[75]*Dy1[11] + Q2[76]*Dy1[12] + Q2[77]*Dy1[13] + Q2[78]*Dy1[14] + Q2[79]*Dy1[15];
QDy1[5] = + Q2[80]*Dy1[0] + Q2[81]*Dy1[1] + Q2[82]*Dy1[2] + Q2[83]*Dy1[3] + Q2[84]*Dy1[4] + Q2[85]*Dy1[5] + Q2[86]*Dy1[6] + Q2[87]*Dy1[7] + Q2[88]*Dy1[8] + Q2[89]*Dy1[9] + Q2[90]*Dy1[10] + Q2[91]*Dy1[11] + Q2[92]*Dy1[12] + Q2[93]*Dy1[13] + Q2[94]*Dy1[14] + Q2[95]*Dy1[15];
QDy1[6] = + Q2[96]*Dy1[0] + Q2[97]*Dy1[1] + Q2[98]*Dy1[2] + Q2[99]*Dy1[3] + Q2[100]*Dy1[4] + Q2[101]*Dy1[5] + Q2[102]*Dy1[6] + Q2[103]*Dy1[7] + Q2[104]*Dy1[8] + Q2[105]*Dy1[9] + Q2[106]*Dy1[10] + Q2[107]*Dy1[11] + Q2[108]*Dy1[12] + Q2[109]*Dy1[13] + Q2[110]*Dy1[14] + Q2[111]*Dy1[15];
QDy1[7] = + Q2[112]*Dy1[0] + Q2[113]*Dy1[1] + Q2[114]*Dy1[2] + Q2[115]*Dy1[3] + Q2[116]*Dy1[4] + Q2[117]*Dy1[5] + Q2[118]*Dy1[6] + Q2[119]*Dy1[7] + Q2[120]*Dy1[8] + Q2[121]*Dy1[9] + Q2[122]*Dy1[10] + Q2[123]*Dy1[11] + Q2[124]*Dy1[12] + Q2[125]*Dy1[13] + Q2[126]*Dy1[14] + Q2[127]*Dy1[15];
QDy1[8] = + Q2[128]*Dy1[0] + Q2[129]*Dy1[1] + Q2[130]*Dy1[2] + Q2[131]*Dy1[3] + Q2[132]*Dy1[4] + Q2[133]*Dy1[5] + Q2[134]*Dy1[6] + Q2[135]*Dy1[7] + Q2[136]*Dy1[8] + Q2[137]*Dy1[9] + Q2[138]*Dy1[10] + Q2[139]*Dy1[11] + Q2[140]*Dy1[12] + Q2[141]*Dy1[13] + Q2[142]*Dy1[14] + Q2[143]*Dy1[15];
QDy1[9] = + Q2[144]*Dy1[0] + Q2[145]*Dy1[1] + Q2[146]*Dy1[2] + Q2[147]*Dy1[3] + Q2[148]*Dy1[4] + Q2[149]*Dy1[5] + Q2[150]*Dy1[6] + Q2[151]*Dy1[7] + Q2[152]*Dy1[8] + Q2[153]*Dy1[9] + Q2[154]*Dy1[10] + Q2[155]*Dy1[11] + Q2[156]*Dy1[12] + Q2[157]*Dy1[13] + Q2[158]*Dy1[14] + Q2[159]*Dy1[15];
QDy1[10] = + Q2[160]*Dy1[0] + Q2[161]*Dy1[1] + Q2[162]*Dy1[2] + Q2[163]*Dy1[3] + Q2[164]*Dy1[4] + Q2[165]*Dy1[5] + Q2[166]*Dy1[6] + Q2[167]*Dy1[7] + Q2[168]*Dy1[8] + Q2[169]*Dy1[9] + Q2[170]*Dy1[10] + Q2[171]*Dy1[11] + Q2[172]*Dy1[12] + Q2[173]*Dy1[13] + Q2[174]*Dy1[14] + Q2[175]*Dy1[15];
QDy1[11] = + Q2[176]*Dy1[0] + Q2[177]*Dy1[1] + Q2[178]*Dy1[2] + Q2[179]*Dy1[3] + Q2[180]*Dy1[4] + Q2[181]*Dy1[5] + Q2[182]*Dy1[6] + Q2[183]*Dy1[7] + Q2[184]*Dy1[8] + Q2[185]*Dy1[9] + Q2[186]*Dy1[10] + Q2[187]*Dy1[11] + Q2[188]*Dy1[12] + Q2[189]*Dy1[13] + Q2[190]*Dy1[14] + Q2[191]*Dy1[15];
}

void acado_condensePrep(  )
{
/* Column: 0 */
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.E, &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 384 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.E[ 432 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 432 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 9, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 384 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.W1, 8, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.E[ 336 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.W1, 7, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1008 ]), &(acadoWorkspace.E[ 288 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.W1, 6, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.E[ 240 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 5, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.E[ 192 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.W1, 4, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 144 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 3, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.E[ 96 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.W1, 2, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 48 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 1, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), acadoWorkspace.E, acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( acadoWorkspace.R1, acadoWorkspace.evGu, acadoWorkspace.W1, 0 );

/* Column: 1 */
acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 528 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.E[ 576 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.E[ 624 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.E[ 672 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.E[ 720 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.E[ 768 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.E[ 816 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.E[ 864 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 864 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 9, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 816 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.W1, 8, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.E[ 768 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.W1, 7, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1008 ]), &(acadoWorkspace.E[ 720 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.W1, 6, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.E[ 672 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 5, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.E[ 624 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.W1, 4, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 576 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 3, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.E[ 528 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.W1, 2, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 480 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 16 ]), &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 1 );

/* Column: 2 */
acado_moveGuE( &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.E[ 912 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.E[ 960 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.E[ 1008 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.E[ 1056 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.E[ 1104 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.E[ 1152 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.E[ 1200 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.E[ 1248 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1248 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 9, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 1200 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.W1, 8, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.E[ 1152 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.W1, 7, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1008 ]), &(acadoWorkspace.E[ 1104 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.W1, 6, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.E[ 1056 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 5, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.E[ 1008 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.W1, 4, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 960 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 3, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.E[ 912 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 32 ]), &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.W1, 2 );

/* Column: 3 */
acado_moveGuE( &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.E[ 1296 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.E[ 1344 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.E[ 1392 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.E[ 1440 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.E[ 1488 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.E[ 1536 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.E[ 1584 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1584 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 9, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 1536 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.W1, 8, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.E[ 1488 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.W1, 7, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1008 ]), &(acadoWorkspace.E[ 1440 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.W1, 6, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.E[ 1392 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 5, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.E[ 1344 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.W1, 4, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 1296 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 48 ]), &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 3 );

/* Column: 4 */
acado_moveGuE( &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.E[ 1632 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.E[ 1680 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.E[ 1728 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.E[ 1776 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.E[ 1824 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.E[ 1872 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1872 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 9, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 1824 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.W1, 8, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.E[ 1776 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.W1, 7, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1008 ]), &(acadoWorkspace.E[ 1728 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.W1, 6, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.E[ 1680 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 5, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.E[ 1632 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 64 ]), &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.W1, 4 );

/* Column: 5 */
acado_moveGuE( &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.E[ 1920 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.E[ 1968 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.E[ 2016 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.E[ 2064 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.E[ 2112 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2112 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 9, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 2064 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.W1, 8, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.E[ 2016 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.W1, 7, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1008 ]), &(acadoWorkspace.E[ 1968 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.W1, 6, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.E[ 1920 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 80 ]), &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 5 );

/* Column: 6 */
acado_moveGuE( &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.E[ 2160 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.E[ 2208 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.E[ 2256 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.E[ 2304 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2304 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 9, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 2256 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.W1, 8, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.E[ 2208 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.W1, 7, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1008 ]), &(acadoWorkspace.E[ 2160 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 96 ]), &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.W1, 6 );

/* Column: 7 */
acado_moveGuE( &(acadoWorkspace.evGu[ 336 ]), &(acadoWorkspace.E[ 2352 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.E[ 2400 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.E[ 2448 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2448 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 9, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 2400 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.W1, 8, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.E[ 2352 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 112 ]), &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.W1, 7 );

/* Column: 8 */
acado_moveGuE( &(acadoWorkspace.evGu[ 384 ]), &(acadoWorkspace.E[ 2496 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.E[ 2544 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2544 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 9, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 2496 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 128 ]), &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.W1, 8 );

/* Column: 9 */
acado_moveGuE( &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.E[ 2592 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2592 ]), acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 144 ]), &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 9 );

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );

acadoWorkspace.sbar[12] = acadoWorkspace.d[0];
acadoWorkspace.sbar[13] = acadoWorkspace.d[1];
acadoWorkspace.sbar[14] = acadoWorkspace.d[2];
acadoWorkspace.sbar[15] = acadoWorkspace.d[3];
acadoWorkspace.sbar[16] = acadoWorkspace.d[4];
acadoWorkspace.sbar[17] = acadoWorkspace.d[5];
acadoWorkspace.sbar[18] = acadoWorkspace.d[6];
acadoWorkspace.sbar[19] = acadoWorkspace.d[7];
acadoWorkspace.sbar[20] = acadoWorkspace.d[8];
acadoWorkspace.sbar[21] = acadoWorkspace.d[9];
acadoWorkspace.sbar[22] = acadoWorkspace.d[10];
acadoWorkspace.sbar[23] = acadoWorkspace.d[11];
acadoWorkspace.sbar[24] = acadoWorkspace.d[12];
acadoWorkspace.sbar[25] = acadoWorkspace.d[13];
acadoWorkspace.sbar[26] = acadoWorkspace.d[14];
acadoWorkspace.sbar[27] = acadoWorkspace.d[15];
acadoWorkspace.sbar[28] = acadoWorkspace.d[16];
acadoWorkspace.sbar[29] = acadoWorkspace.d[17];
acadoWorkspace.sbar[30] = acadoWorkspace.d[18];
acadoWorkspace.sbar[31] = acadoWorkspace.d[19];
acadoWorkspace.sbar[32] = acadoWorkspace.d[20];
acadoWorkspace.sbar[33] = acadoWorkspace.d[21];
acadoWorkspace.sbar[34] = acadoWorkspace.d[22];
acadoWorkspace.sbar[35] = acadoWorkspace.d[23];
acadoWorkspace.sbar[36] = acadoWorkspace.d[24];
acadoWorkspace.sbar[37] = acadoWorkspace.d[25];
acadoWorkspace.sbar[38] = acadoWorkspace.d[26];
acadoWorkspace.sbar[39] = acadoWorkspace.d[27];
acadoWorkspace.sbar[40] = acadoWorkspace.d[28];
acadoWorkspace.sbar[41] = acadoWorkspace.d[29];
acadoWorkspace.sbar[42] = acadoWorkspace.d[30];
acadoWorkspace.sbar[43] = acadoWorkspace.d[31];
acadoWorkspace.sbar[44] = acadoWorkspace.d[32];
acadoWorkspace.sbar[45] = acadoWorkspace.d[33];
acadoWorkspace.sbar[46] = acadoWorkspace.d[34];
acadoWorkspace.sbar[47] = acadoWorkspace.d[35];
acadoWorkspace.sbar[48] = acadoWorkspace.d[36];
acadoWorkspace.sbar[49] = acadoWorkspace.d[37];
acadoWorkspace.sbar[50] = acadoWorkspace.d[38];
acadoWorkspace.sbar[51] = acadoWorkspace.d[39];
acadoWorkspace.sbar[52] = acadoWorkspace.d[40];
acadoWorkspace.sbar[53] = acadoWorkspace.d[41];
acadoWorkspace.sbar[54] = acadoWorkspace.d[42];
acadoWorkspace.sbar[55] = acadoWorkspace.d[43];
acadoWorkspace.sbar[56] = acadoWorkspace.d[44];
acadoWorkspace.sbar[57] = acadoWorkspace.d[45];
acadoWorkspace.sbar[58] = acadoWorkspace.d[46];
acadoWorkspace.sbar[59] = acadoWorkspace.d[47];
acadoWorkspace.sbar[60] = acadoWorkspace.d[48];
acadoWorkspace.sbar[61] = acadoWorkspace.d[49];
acadoWorkspace.sbar[62] = acadoWorkspace.d[50];
acadoWorkspace.sbar[63] = acadoWorkspace.d[51];
acadoWorkspace.sbar[64] = acadoWorkspace.d[52];
acadoWorkspace.sbar[65] = acadoWorkspace.d[53];
acadoWorkspace.sbar[66] = acadoWorkspace.d[54];
acadoWorkspace.sbar[67] = acadoWorkspace.d[55];
acadoWorkspace.sbar[68] = acadoWorkspace.d[56];
acadoWorkspace.sbar[69] = acadoWorkspace.d[57];
acadoWorkspace.sbar[70] = acadoWorkspace.d[58];
acadoWorkspace.sbar[71] = acadoWorkspace.d[59];
acadoWorkspace.sbar[72] = acadoWorkspace.d[60];
acadoWorkspace.sbar[73] = acadoWorkspace.d[61];
acadoWorkspace.sbar[74] = acadoWorkspace.d[62];
acadoWorkspace.sbar[75] = acadoWorkspace.d[63];
acadoWorkspace.sbar[76] = acadoWorkspace.d[64];
acadoWorkspace.sbar[77] = acadoWorkspace.d[65];
acadoWorkspace.sbar[78] = acadoWorkspace.d[66];
acadoWorkspace.sbar[79] = acadoWorkspace.d[67];
acadoWorkspace.sbar[80] = acadoWorkspace.d[68];
acadoWorkspace.sbar[81] = acadoWorkspace.d[69];
acadoWorkspace.sbar[82] = acadoWorkspace.d[70];
acadoWorkspace.sbar[83] = acadoWorkspace.d[71];
acadoWorkspace.sbar[84] = acadoWorkspace.d[72];
acadoWorkspace.sbar[85] = acadoWorkspace.d[73];
acadoWorkspace.sbar[86] = acadoWorkspace.d[74];
acadoWorkspace.sbar[87] = acadoWorkspace.d[75];
acadoWorkspace.sbar[88] = acadoWorkspace.d[76];
acadoWorkspace.sbar[89] = acadoWorkspace.d[77];
acadoWorkspace.sbar[90] = acadoWorkspace.d[78];
acadoWorkspace.sbar[91] = acadoWorkspace.d[79];
acadoWorkspace.sbar[92] = acadoWorkspace.d[80];
acadoWorkspace.sbar[93] = acadoWorkspace.d[81];
acadoWorkspace.sbar[94] = acadoWorkspace.d[82];
acadoWorkspace.sbar[95] = acadoWorkspace.d[83];
acadoWorkspace.sbar[96] = acadoWorkspace.d[84];
acadoWorkspace.sbar[97] = acadoWorkspace.d[85];
acadoWorkspace.sbar[98] = acadoWorkspace.d[86];
acadoWorkspace.sbar[99] = acadoWorkspace.d[87];
acadoWorkspace.sbar[100] = acadoWorkspace.d[88];
acadoWorkspace.sbar[101] = acadoWorkspace.d[89];
acadoWorkspace.sbar[102] = acadoWorkspace.d[90];
acadoWorkspace.sbar[103] = acadoWorkspace.d[91];
acadoWorkspace.sbar[104] = acadoWorkspace.d[92];
acadoWorkspace.sbar[105] = acadoWorkspace.d[93];
acadoWorkspace.sbar[106] = acadoWorkspace.d[94];
acadoWorkspace.sbar[107] = acadoWorkspace.d[95];
acadoWorkspace.sbar[108] = acadoWorkspace.d[96];
acadoWorkspace.sbar[109] = acadoWorkspace.d[97];
acadoWorkspace.sbar[110] = acadoWorkspace.d[98];
acadoWorkspace.sbar[111] = acadoWorkspace.d[99];
acadoWorkspace.sbar[112] = acadoWorkspace.d[100];
acadoWorkspace.sbar[113] = acadoWorkspace.d[101];
acadoWorkspace.sbar[114] = acadoWorkspace.d[102];
acadoWorkspace.sbar[115] = acadoWorkspace.d[103];
acadoWorkspace.sbar[116] = acadoWorkspace.d[104];
acadoWorkspace.sbar[117] = acadoWorkspace.d[105];
acadoWorkspace.sbar[118] = acadoWorkspace.d[106];
acadoWorkspace.sbar[119] = acadoWorkspace.d[107];
acadoWorkspace.sbar[120] = acadoWorkspace.d[108];
acadoWorkspace.sbar[121] = acadoWorkspace.d[109];
acadoWorkspace.sbar[122] = acadoWorkspace.d[110];
acadoWorkspace.sbar[123] = acadoWorkspace.d[111];
acadoWorkspace.sbar[124] = acadoWorkspace.d[112];
acadoWorkspace.sbar[125] = acadoWorkspace.d[113];
acadoWorkspace.sbar[126] = acadoWorkspace.d[114];
acadoWorkspace.sbar[127] = acadoWorkspace.d[115];
acadoWorkspace.sbar[128] = acadoWorkspace.d[116];
acadoWorkspace.sbar[129] = acadoWorkspace.d[117];
acadoWorkspace.sbar[130] = acadoWorkspace.d[118];
acadoWorkspace.sbar[131] = acadoWorkspace.d[119];

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
acadoWorkspace.Dx0[10] = acadoVariables.x0[10] - acadoVariables.x[10];
acadoWorkspace.Dx0[11] = acadoVariables.x0[11] - acadoVariables.x[11];
for (lRun1 = 0; lRun1 < 160; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];
acadoWorkspace.DyN[7] -= acadoVariables.yN[7];
acadoWorkspace.DyN[8] -= acadoVariables.yN[8];
acadoWorkspace.DyN[9] -= acadoVariables.yN[9];
acadoWorkspace.DyN[10] -= acadoVariables.yN[10];
acadoWorkspace.DyN[11] -= acadoVariables.yN[11];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 64 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 128 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 192 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 256 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 320 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 384 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 448 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 512 ]), &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 576 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 36 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 192 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 384 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 576 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 768 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 960 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1152 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1344 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1536 ]), &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1728 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 108 ]) );

acadoWorkspace.QDy[120] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[121] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[122] = + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[123] = + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[124] = + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[125] = + acadoWorkspace.QN2[60]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[61]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[62]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[63]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[64]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[65]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[66]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[67]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[68]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[69]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[70]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[71]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[126] = + acadoWorkspace.QN2[72]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[73]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[74]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[75]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[76]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[77]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[78]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[79]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[80]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[81]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[82]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[83]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[127] = + acadoWorkspace.QN2[84]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[85]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[86]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[87]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[88]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[89]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[90]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[91]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[92]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[93]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[94]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[95]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[128] = + acadoWorkspace.QN2[96]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[97]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[98]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[99]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[100]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[101]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[102]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[103]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[104]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[105]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[106]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[107]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[129] = + acadoWorkspace.QN2[108]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[109]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[110]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[111]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[112]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[113]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[114]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[115]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[116]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[117]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[118]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[119]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[130] = + acadoWorkspace.QN2[120]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[121]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[122]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[123]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[124]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[125]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[126]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[127]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[128]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[129]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[130]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[131]*acadoWorkspace.DyN[11];
acadoWorkspace.QDy[131] = + acadoWorkspace.QN2[132]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[133]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[134]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[135]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[136]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[137]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[138]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[139]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[140]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[141]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[142]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[143]*acadoWorkspace.DyN[11];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 120 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[120];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[121];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[122];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[123];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[124];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[125];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[81]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[82]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[83]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[126];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[84]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[85]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[86]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[87]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[88]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[89]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[90]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[91]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[92]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[93]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[94]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[95]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[127];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[96]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[97]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[98]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[99]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[100]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[101]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[102]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[103]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[104]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[105]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[106]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[107]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[128];
acadoWorkspace.w1[9] = + acadoWorkspace.QN1[108]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[109]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[110]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[111]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[112]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[113]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[114]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[115]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[116]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[117]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[118]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[119]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[129];
acadoWorkspace.w1[10] = + acadoWorkspace.QN1[120]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[121]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[122]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[123]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[124]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[125]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[126]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[127]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[128]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[129]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[130]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[131]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[130];
acadoWorkspace.w1[11] = + acadoWorkspace.QN1[132]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[133]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[134]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[135]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[136]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[137]*acadoWorkspace.sbar[125] + acadoWorkspace.QN1[138]*acadoWorkspace.sbar[126] + acadoWorkspace.QN1[139]*acadoWorkspace.sbar[127] + acadoWorkspace.QN1[140]*acadoWorkspace.sbar[128] + acadoWorkspace.QN1[141]*acadoWorkspace.sbar[129] + acadoWorkspace.QN1[142]*acadoWorkspace.sbar[130] + acadoWorkspace.QN1[143]*acadoWorkspace.sbar[131] + acadoWorkspace.QDy[131];
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1008 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acadoWorkspace.sbar[12] = acadoWorkspace.d[0];
acadoWorkspace.sbar[13] = acadoWorkspace.d[1];
acadoWorkspace.sbar[14] = acadoWorkspace.d[2];
acadoWorkspace.sbar[15] = acadoWorkspace.d[3];
acadoWorkspace.sbar[16] = acadoWorkspace.d[4];
acadoWorkspace.sbar[17] = acadoWorkspace.d[5];
acadoWorkspace.sbar[18] = acadoWorkspace.d[6];
acadoWorkspace.sbar[19] = acadoWorkspace.d[7];
acadoWorkspace.sbar[20] = acadoWorkspace.d[8];
acadoWorkspace.sbar[21] = acadoWorkspace.d[9];
acadoWorkspace.sbar[22] = acadoWorkspace.d[10];
acadoWorkspace.sbar[23] = acadoWorkspace.d[11];
acadoWorkspace.sbar[24] = acadoWorkspace.d[12];
acadoWorkspace.sbar[25] = acadoWorkspace.d[13];
acadoWorkspace.sbar[26] = acadoWorkspace.d[14];
acadoWorkspace.sbar[27] = acadoWorkspace.d[15];
acadoWorkspace.sbar[28] = acadoWorkspace.d[16];
acadoWorkspace.sbar[29] = acadoWorkspace.d[17];
acadoWorkspace.sbar[30] = acadoWorkspace.d[18];
acadoWorkspace.sbar[31] = acadoWorkspace.d[19];
acadoWorkspace.sbar[32] = acadoWorkspace.d[20];
acadoWorkspace.sbar[33] = acadoWorkspace.d[21];
acadoWorkspace.sbar[34] = acadoWorkspace.d[22];
acadoWorkspace.sbar[35] = acadoWorkspace.d[23];
acadoWorkspace.sbar[36] = acadoWorkspace.d[24];
acadoWorkspace.sbar[37] = acadoWorkspace.d[25];
acadoWorkspace.sbar[38] = acadoWorkspace.d[26];
acadoWorkspace.sbar[39] = acadoWorkspace.d[27];
acadoWorkspace.sbar[40] = acadoWorkspace.d[28];
acadoWorkspace.sbar[41] = acadoWorkspace.d[29];
acadoWorkspace.sbar[42] = acadoWorkspace.d[30];
acadoWorkspace.sbar[43] = acadoWorkspace.d[31];
acadoWorkspace.sbar[44] = acadoWorkspace.d[32];
acadoWorkspace.sbar[45] = acadoWorkspace.d[33];
acadoWorkspace.sbar[46] = acadoWorkspace.d[34];
acadoWorkspace.sbar[47] = acadoWorkspace.d[35];
acadoWorkspace.sbar[48] = acadoWorkspace.d[36];
acadoWorkspace.sbar[49] = acadoWorkspace.d[37];
acadoWorkspace.sbar[50] = acadoWorkspace.d[38];
acadoWorkspace.sbar[51] = acadoWorkspace.d[39];
acadoWorkspace.sbar[52] = acadoWorkspace.d[40];
acadoWorkspace.sbar[53] = acadoWorkspace.d[41];
acadoWorkspace.sbar[54] = acadoWorkspace.d[42];
acadoWorkspace.sbar[55] = acadoWorkspace.d[43];
acadoWorkspace.sbar[56] = acadoWorkspace.d[44];
acadoWorkspace.sbar[57] = acadoWorkspace.d[45];
acadoWorkspace.sbar[58] = acadoWorkspace.d[46];
acadoWorkspace.sbar[59] = acadoWorkspace.d[47];
acadoWorkspace.sbar[60] = acadoWorkspace.d[48];
acadoWorkspace.sbar[61] = acadoWorkspace.d[49];
acadoWorkspace.sbar[62] = acadoWorkspace.d[50];
acadoWorkspace.sbar[63] = acadoWorkspace.d[51];
acadoWorkspace.sbar[64] = acadoWorkspace.d[52];
acadoWorkspace.sbar[65] = acadoWorkspace.d[53];
acadoWorkspace.sbar[66] = acadoWorkspace.d[54];
acadoWorkspace.sbar[67] = acadoWorkspace.d[55];
acadoWorkspace.sbar[68] = acadoWorkspace.d[56];
acadoWorkspace.sbar[69] = acadoWorkspace.d[57];
acadoWorkspace.sbar[70] = acadoWorkspace.d[58];
acadoWorkspace.sbar[71] = acadoWorkspace.d[59];
acadoWorkspace.sbar[72] = acadoWorkspace.d[60];
acadoWorkspace.sbar[73] = acadoWorkspace.d[61];
acadoWorkspace.sbar[74] = acadoWorkspace.d[62];
acadoWorkspace.sbar[75] = acadoWorkspace.d[63];
acadoWorkspace.sbar[76] = acadoWorkspace.d[64];
acadoWorkspace.sbar[77] = acadoWorkspace.d[65];
acadoWorkspace.sbar[78] = acadoWorkspace.d[66];
acadoWorkspace.sbar[79] = acadoWorkspace.d[67];
acadoWorkspace.sbar[80] = acadoWorkspace.d[68];
acadoWorkspace.sbar[81] = acadoWorkspace.d[69];
acadoWorkspace.sbar[82] = acadoWorkspace.d[70];
acadoWorkspace.sbar[83] = acadoWorkspace.d[71];
acadoWorkspace.sbar[84] = acadoWorkspace.d[72];
acadoWorkspace.sbar[85] = acadoWorkspace.d[73];
acadoWorkspace.sbar[86] = acadoWorkspace.d[74];
acadoWorkspace.sbar[87] = acadoWorkspace.d[75];
acadoWorkspace.sbar[88] = acadoWorkspace.d[76];
acadoWorkspace.sbar[89] = acadoWorkspace.d[77];
acadoWorkspace.sbar[90] = acadoWorkspace.d[78];
acadoWorkspace.sbar[91] = acadoWorkspace.d[79];
acadoWorkspace.sbar[92] = acadoWorkspace.d[80];
acadoWorkspace.sbar[93] = acadoWorkspace.d[81];
acadoWorkspace.sbar[94] = acadoWorkspace.d[82];
acadoWorkspace.sbar[95] = acadoWorkspace.d[83];
acadoWorkspace.sbar[96] = acadoWorkspace.d[84];
acadoWorkspace.sbar[97] = acadoWorkspace.d[85];
acadoWorkspace.sbar[98] = acadoWorkspace.d[86];
acadoWorkspace.sbar[99] = acadoWorkspace.d[87];
acadoWorkspace.sbar[100] = acadoWorkspace.d[88];
acadoWorkspace.sbar[101] = acadoWorkspace.d[89];
acadoWorkspace.sbar[102] = acadoWorkspace.d[90];
acadoWorkspace.sbar[103] = acadoWorkspace.d[91];
acadoWorkspace.sbar[104] = acadoWorkspace.d[92];
acadoWorkspace.sbar[105] = acadoWorkspace.d[93];
acadoWorkspace.sbar[106] = acadoWorkspace.d[94];
acadoWorkspace.sbar[107] = acadoWorkspace.d[95];
acadoWorkspace.sbar[108] = acadoWorkspace.d[96];
acadoWorkspace.sbar[109] = acadoWorkspace.d[97];
acadoWorkspace.sbar[110] = acadoWorkspace.d[98];
acadoWorkspace.sbar[111] = acadoWorkspace.d[99];
acadoWorkspace.sbar[112] = acadoWorkspace.d[100];
acadoWorkspace.sbar[113] = acadoWorkspace.d[101];
acadoWorkspace.sbar[114] = acadoWorkspace.d[102];
acadoWorkspace.sbar[115] = acadoWorkspace.d[103];
acadoWorkspace.sbar[116] = acadoWorkspace.d[104];
acadoWorkspace.sbar[117] = acadoWorkspace.d[105];
acadoWorkspace.sbar[118] = acadoWorkspace.d[106];
acadoWorkspace.sbar[119] = acadoWorkspace.d[107];
acadoWorkspace.sbar[120] = acadoWorkspace.d[108];
acadoWorkspace.sbar[121] = acadoWorkspace.d[109];
acadoWorkspace.sbar[122] = acadoWorkspace.d[110];
acadoWorkspace.sbar[123] = acadoWorkspace.d[111];
acadoWorkspace.sbar[124] = acadoWorkspace.d[112];
acadoWorkspace.sbar[125] = acadoWorkspace.d[113];
acadoWorkspace.sbar[126] = acadoWorkspace.d[114];
acadoWorkspace.sbar[127] = acadoWorkspace.d[115];
acadoWorkspace.sbar[128] = acadoWorkspace.d[116];
acadoWorkspace.sbar[129] = acadoWorkspace.d[117];
acadoWorkspace.sbar[130] = acadoWorkspace.d[118];
acadoWorkspace.sbar[131] = acadoWorkspace.d[119];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.evGu[ 336 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.evGu[ 384 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 120 ]) );
for (lRun1 = 0; lRun1 < 132; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -1.0000000000000000e+00;
acadoVariables.lbValues[1] = -1.0000000000000000e+00;
acadoVariables.lbValues[2] = -1.0000000000000000e+00;
acadoVariables.lbValues[3] = 1.0000000000000001e-01;
acadoVariables.lbValues[4] = -1.0000000000000000e+00;
acadoVariables.lbValues[5] = -1.0000000000000000e+00;
acadoVariables.lbValues[6] = -1.0000000000000000e+00;
acadoVariables.lbValues[7] = 1.0000000000000001e-01;
acadoVariables.lbValues[8] = -1.0000000000000000e+00;
acadoVariables.lbValues[9] = -1.0000000000000000e+00;
acadoVariables.lbValues[10] = -1.0000000000000000e+00;
acadoVariables.lbValues[11] = 1.0000000000000001e-01;
acadoVariables.lbValues[12] = -1.0000000000000000e+00;
acadoVariables.lbValues[13] = -1.0000000000000000e+00;
acadoVariables.lbValues[14] = -1.0000000000000000e+00;
acadoVariables.lbValues[15] = 1.0000000000000001e-01;
acadoVariables.lbValues[16] = -1.0000000000000000e+00;
acadoVariables.lbValues[17] = -1.0000000000000000e+00;
acadoVariables.lbValues[18] = -1.0000000000000000e+00;
acadoVariables.lbValues[19] = 1.0000000000000001e-01;
acadoVariables.lbValues[20] = -1.0000000000000000e+00;
acadoVariables.lbValues[21] = -1.0000000000000000e+00;
acadoVariables.lbValues[22] = -1.0000000000000000e+00;
acadoVariables.lbValues[23] = 1.0000000000000001e-01;
acadoVariables.lbValues[24] = -1.0000000000000000e+00;
acadoVariables.lbValues[25] = -1.0000000000000000e+00;
acadoVariables.lbValues[26] = -1.0000000000000000e+00;
acadoVariables.lbValues[27] = 1.0000000000000001e-01;
acadoVariables.lbValues[28] = -1.0000000000000000e+00;
acadoVariables.lbValues[29] = -1.0000000000000000e+00;
acadoVariables.lbValues[30] = -1.0000000000000000e+00;
acadoVariables.lbValues[31] = 1.0000000000000001e-01;
acadoVariables.lbValues[32] = -1.0000000000000000e+00;
acadoVariables.lbValues[33] = -1.0000000000000000e+00;
acadoVariables.lbValues[34] = -1.0000000000000000e+00;
acadoVariables.lbValues[35] = 1.0000000000000001e-01;
acadoVariables.lbValues[36] = -1.0000000000000000e+00;
acadoVariables.lbValues[37] = -1.0000000000000000e+00;
acadoVariables.lbValues[38] = -1.0000000000000000e+00;
acadoVariables.lbValues[39] = 1.0000000000000001e-01;
acadoVariables.ubValues[0] = 1.0000000000000000e+00;
acadoVariables.ubValues[1] = 1.0000000000000000e+00;
acadoVariables.ubValues[2] = 1.0000000000000000e+00;
acadoVariables.ubValues[3] = 3.0000000000000000e+01;
acadoVariables.ubValues[4] = 1.0000000000000000e+00;
acadoVariables.ubValues[5] = 1.0000000000000000e+00;
acadoVariables.ubValues[6] = 1.0000000000000000e+00;
acadoVariables.ubValues[7] = 3.0000000000000000e+01;
acadoVariables.ubValues[8] = 1.0000000000000000e+00;
acadoVariables.ubValues[9] = 1.0000000000000000e+00;
acadoVariables.ubValues[10] = 1.0000000000000000e+00;
acadoVariables.ubValues[11] = 3.0000000000000000e+01;
acadoVariables.ubValues[12] = 1.0000000000000000e+00;
acadoVariables.ubValues[13] = 1.0000000000000000e+00;
acadoVariables.ubValues[14] = 1.0000000000000000e+00;
acadoVariables.ubValues[15] = 3.0000000000000000e+01;
acadoVariables.ubValues[16] = 1.0000000000000000e+00;
acadoVariables.ubValues[17] = 1.0000000000000000e+00;
acadoVariables.ubValues[18] = 1.0000000000000000e+00;
acadoVariables.ubValues[19] = 3.0000000000000000e+01;
acadoVariables.ubValues[20] = 1.0000000000000000e+00;
acadoVariables.ubValues[21] = 1.0000000000000000e+00;
acadoVariables.ubValues[22] = 1.0000000000000000e+00;
acadoVariables.ubValues[23] = 3.0000000000000000e+01;
acadoVariables.ubValues[24] = 1.0000000000000000e+00;
acadoVariables.ubValues[25] = 1.0000000000000000e+00;
acadoVariables.ubValues[26] = 1.0000000000000000e+00;
acadoVariables.ubValues[27] = 3.0000000000000000e+01;
acadoVariables.ubValues[28] = 1.0000000000000000e+00;
acadoVariables.ubValues[29] = 1.0000000000000000e+00;
acadoVariables.ubValues[30] = 1.0000000000000000e+00;
acadoVariables.ubValues[31] = 3.0000000000000000e+01;
acadoVariables.ubValues[32] = 1.0000000000000000e+00;
acadoVariables.ubValues[33] = 1.0000000000000000e+00;
acadoVariables.ubValues[34] = 1.0000000000000000e+00;
acadoVariables.ubValues[35] = 3.0000000000000000e+01;
acadoVariables.ubValues[36] = 1.0000000000000000e+00;
acadoVariables.ubValues[37] = 1.0000000000000000e+00;
acadoVariables.ubValues[38] = 1.0000000000000000e+00;
acadoVariables.ubValues[39] = 3.0000000000000000e+01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
state[0] = acadoVariables.x[index * 12];
state[1] = acadoVariables.x[index * 12 + 1];
state[2] = acadoVariables.x[index * 12 + 2];
state[3] = acadoVariables.x[index * 12 + 3];
state[4] = acadoVariables.x[index * 12 + 4];
state[5] = acadoVariables.x[index * 12 + 5];
state[6] = acadoVariables.x[index * 12 + 6];
state[7] = acadoVariables.x[index * 12 + 7];
state[8] = acadoVariables.x[index * 12 + 8];
state[9] = acadoVariables.x[index * 12 + 9];
state[10] = acadoVariables.x[index * 12 + 10];
state[11] = acadoVariables.x[index * 12 + 11];
state[204] = acadoVariables.u[index * 4];
state[205] = acadoVariables.u[index * 4 + 1];
state[206] = acadoVariables.u[index * 4 + 2];
state[207] = acadoVariables.u[index * 4 + 3];

acado_integrate(state, index == 0);

acadoVariables.x[index * 12 + 12] = state[0];
acadoVariables.x[index * 12 + 13] = state[1];
acadoVariables.x[index * 12 + 14] = state[2];
acadoVariables.x[index * 12 + 15] = state[3];
acadoVariables.x[index * 12 + 16] = state[4];
acadoVariables.x[index * 12 + 17] = state[5];
acadoVariables.x[index * 12 + 18] = state[6];
acadoVariables.x[index * 12 + 19] = state[7];
acadoVariables.x[index * 12 + 20] = state[8];
acadoVariables.x[index * 12 + 21] = state[9];
acadoVariables.x[index * 12 + 22] = state[10];
acadoVariables.x[index * 12 + 23] = state[11];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 12] = acadoVariables.x[index * 12 + 12];
acadoVariables.x[index * 12 + 1] = acadoVariables.x[index * 12 + 13];
acadoVariables.x[index * 12 + 2] = acadoVariables.x[index * 12 + 14];
acadoVariables.x[index * 12 + 3] = acadoVariables.x[index * 12 + 15];
acadoVariables.x[index * 12 + 4] = acadoVariables.x[index * 12 + 16];
acadoVariables.x[index * 12 + 5] = acadoVariables.x[index * 12 + 17];
acadoVariables.x[index * 12 + 6] = acadoVariables.x[index * 12 + 18];
acadoVariables.x[index * 12 + 7] = acadoVariables.x[index * 12 + 19];
acadoVariables.x[index * 12 + 8] = acadoVariables.x[index * 12 + 20];
acadoVariables.x[index * 12 + 9] = acadoVariables.x[index * 12 + 21];
acadoVariables.x[index * 12 + 10] = acadoVariables.x[index * 12 + 22];
acadoVariables.x[index * 12 + 11] = acadoVariables.x[index * 12 + 23];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[120] = xEnd[0];
acadoVariables.x[121] = xEnd[1];
acadoVariables.x[122] = xEnd[2];
acadoVariables.x[123] = xEnd[3];
acadoVariables.x[124] = xEnd[4];
acadoVariables.x[125] = xEnd[5];
acadoVariables.x[126] = xEnd[6];
acadoVariables.x[127] = xEnd[7];
acadoVariables.x[128] = xEnd[8];
acadoVariables.x[129] = xEnd[9];
acadoVariables.x[130] = xEnd[10];
acadoVariables.x[131] = xEnd[11];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[120];
state[1] = acadoVariables.x[121];
state[2] = acadoVariables.x[122];
state[3] = acadoVariables.x[123];
state[4] = acadoVariables.x[124];
state[5] = acadoVariables.x[125];
state[6] = acadoVariables.x[126];
state[7] = acadoVariables.x[127];
state[8] = acadoVariables.x[128];
state[9] = acadoVariables.x[129];
state[10] = acadoVariables.x[130];
state[11] = acadoVariables.x[131];
if (uEnd != 0)
{
state[204] = uEnd[0];
state[205] = uEnd[1];
state[206] = uEnd[2];
state[207] = uEnd[3];
}
else
{
state[204] = acadoVariables.u[36];
state[205] = acadoVariables.u[37];
state[206] = acadoVariables.u[38];
state[207] = acadoVariables.u[39];
}

acado_integrate(state, 1);

acadoVariables.x[120] = state[0];
acadoVariables.x[121] = state[1];
acadoVariables.x[122] = state[2];
acadoVariables.x[123] = state[3];
acadoVariables.x[124] = state[4];
acadoVariables.x[125] = state[5];
acadoVariables.x[126] = state[6];
acadoVariables.x[127] = state[7];
acadoVariables.x[128] = state[8];
acadoVariables.x[129] = state[9];
acadoVariables.x[130] = state[10];
acadoVariables.x[131] = state[11];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[36] = uEnd[0];
acadoVariables.u[37] = uEnd[1];
acadoVariables.u[38] = uEnd[2];
acadoVariables.u[39] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39];
kkt = fabs( kkt );
for (index = 0; index < 40; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 16 */
real_t tmpDy[ 16 ];

/** Row vector of size: 12 */
real_t tmpDyN[ 12 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 12];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 12 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 12 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 12 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 12 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 12 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 12 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 12 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 12 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 12 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[lRun1 * 12 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[lRun1 * 12 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[13] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.u[lRun1 * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 16] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 16];
acadoWorkspace.Dy[lRun1 * 16 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 16 + 1];
acadoWorkspace.Dy[lRun1 * 16 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 16 + 2];
acadoWorkspace.Dy[lRun1 * 16 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 16 + 3];
acadoWorkspace.Dy[lRun1 * 16 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 16 + 4];
acadoWorkspace.Dy[lRun1 * 16 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 16 + 5];
acadoWorkspace.Dy[lRun1 * 16 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 16 + 6];
acadoWorkspace.Dy[lRun1 * 16 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 16 + 7];
acadoWorkspace.Dy[lRun1 * 16 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 16 + 8];
acadoWorkspace.Dy[lRun1 * 16 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 16 + 9];
acadoWorkspace.Dy[lRun1 * 16 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 16 + 10];
acadoWorkspace.Dy[lRun1 * 16 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 16 + 11];
acadoWorkspace.Dy[lRun1 * 16 + 12] = acadoWorkspace.objValueOut[12] - acadoVariables.y[lRun1 * 16 + 12];
acadoWorkspace.Dy[lRun1 * 16 + 13] = acadoWorkspace.objValueOut[13] - acadoVariables.y[lRun1 * 16 + 13];
acadoWorkspace.Dy[lRun1 * 16 + 14] = acadoWorkspace.objValueOut[14] - acadoVariables.y[lRun1 * 16 + 14];
acadoWorkspace.Dy[lRun1 * 16 + 15] = acadoWorkspace.objValueOut[15] - acadoVariables.y[lRun1 * 16 + 15];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.x[124];
acadoWorkspace.objValueIn[5] = acadoVariables.x[125];
acadoWorkspace.objValueIn[6] = acadoVariables.x[126];
acadoWorkspace.objValueIn[7] = acadoVariables.x[127];
acadoWorkspace.objValueIn[8] = acadoVariables.x[128];
acadoWorkspace.objValueIn[9] = acadoVariables.x[129];
acadoWorkspace.objValueIn[10] = acadoVariables.x[130];
acadoWorkspace.objValueIn[11] = acadoVariables.x[131];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7] - acadoVariables.yN[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8] - acadoVariables.yN[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9] - acadoVariables.yN[9];
acadoWorkspace.DyN[10] = acadoWorkspace.objValueOut[10] - acadoVariables.yN[10];
acadoWorkspace.DyN[11] = acadoWorkspace.objValueOut[11] - acadoVariables.yN[11];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 16] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 32] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 48] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 64] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 80] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 96] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 112] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 128] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 144] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 160] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 176] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 192] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 208] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 224] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 240];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 1] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 17] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 33] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 49] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 65] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 81] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 97] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 113] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 129] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 145] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 161] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 177] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 193] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 209] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 225] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 241];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 2] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 18] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 34] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 50] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 66] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 82] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 98] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 114] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 130] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 146] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 162] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 178] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 194] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 210] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 226] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 242];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 3] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 19] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 35] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 51] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 67] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 83] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 99] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 115] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 131] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 147] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 163] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 179] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 195] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 211] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 227] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 243];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 4] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 20] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 36] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 52] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 68] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 84] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 100] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 116] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 132] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 148] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 164] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 180] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 196] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 212] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 228] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 244];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 5] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 21] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 37] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 53] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 69] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 85] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 101] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 117] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 133] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 149] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 165] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 181] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 197] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 213] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 229] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 245];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 6] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 22] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 38] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 54] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 70] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 86] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 102] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 118] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 134] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 150] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 166] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 182] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 198] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 214] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 230] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 246];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 7] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 23] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 39] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 55] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 71] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 87] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 103] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 119] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 135] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 151] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 167] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 183] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 199] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 215] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 231] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 247];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 8] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 24] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 40] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 56] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 72] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 88] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 104] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 120] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 136] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 152] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 168] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 184] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 200] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 216] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 232] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 248];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 9] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 25] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 41] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 57] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 73] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 89] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 105] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 121] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 137] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 153] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 169] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 185] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 201] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 217] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 233] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 249];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 10] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 26] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 42] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 58] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 74] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 90] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 106] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 122] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 138] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 154] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 170] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 186] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 202] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 218] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 234] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 250];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 11] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 27] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 43] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 59] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 75] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 91] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 107] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 123] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 139] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 155] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 171] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 187] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 203] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 219] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 235] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 251];
tmpDy[12] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 12] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 28] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 44] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 60] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 76] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 92] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 108] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 124] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 140] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 156] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 172] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 188] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 204] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 220] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 236] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 252];
tmpDy[13] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 13] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 29] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 45] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 61] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 77] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 93] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 109] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 125] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 141] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 157] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 173] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 189] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 205] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 221] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 237] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 253];
tmpDy[14] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 14] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 30] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 46] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 62] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 78] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 94] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 110] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 126] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 142] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 158] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 174] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 190] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 206] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 222] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 238] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 254];
tmpDy[15] = + acadoWorkspace.Dy[lRun1 * 16]*acadoVariables.W[lRun1 * 256 + 15] + acadoWorkspace.Dy[lRun1 * 16 + 1]*acadoVariables.W[lRun1 * 256 + 31] + acadoWorkspace.Dy[lRun1 * 16 + 2]*acadoVariables.W[lRun1 * 256 + 47] + acadoWorkspace.Dy[lRun1 * 16 + 3]*acadoVariables.W[lRun1 * 256 + 63] + acadoWorkspace.Dy[lRun1 * 16 + 4]*acadoVariables.W[lRun1 * 256 + 79] + acadoWorkspace.Dy[lRun1 * 16 + 5]*acadoVariables.W[lRun1 * 256 + 95] + acadoWorkspace.Dy[lRun1 * 16 + 6]*acadoVariables.W[lRun1 * 256 + 111] + acadoWorkspace.Dy[lRun1 * 16 + 7]*acadoVariables.W[lRun1 * 256 + 127] + acadoWorkspace.Dy[lRun1 * 16 + 8]*acadoVariables.W[lRun1 * 256 + 143] + acadoWorkspace.Dy[lRun1 * 16 + 9]*acadoVariables.W[lRun1 * 256 + 159] + acadoWorkspace.Dy[lRun1 * 16 + 10]*acadoVariables.W[lRun1 * 256 + 175] + acadoWorkspace.Dy[lRun1 * 16 + 11]*acadoVariables.W[lRun1 * 256 + 191] + acadoWorkspace.Dy[lRun1 * 16 + 12]*acadoVariables.W[lRun1 * 256 + 207] + acadoWorkspace.Dy[lRun1 * 16 + 13]*acadoVariables.W[lRun1 * 256 + 223] + acadoWorkspace.Dy[lRun1 * 16 + 14]*acadoVariables.W[lRun1 * 256 + 239] + acadoWorkspace.Dy[lRun1 * 16 + 15]*acadoVariables.W[lRun1 * 256 + 255];
objVal += + acadoWorkspace.Dy[lRun1 * 16]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 16 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 16 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 16 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 16 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 16 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 16 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 16 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 16 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 16 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 16 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 16 + 11]*tmpDy[11] + acadoWorkspace.Dy[lRun1 * 16 + 12]*tmpDy[12] + acadoWorkspace.Dy[lRun1 * 16 + 13]*tmpDy[13] + acadoWorkspace.Dy[lRun1 * 16 + 14]*tmpDy[14] + acadoWorkspace.Dy[lRun1 * 16 + 15]*tmpDy[15];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[13];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[26];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[39];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[52];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[65];
tmpDyN[6] = + acadoWorkspace.DyN[6]*acadoVariables.WN[78];
tmpDyN[7] = + acadoWorkspace.DyN[7]*acadoVariables.WN[91];
tmpDyN[8] = + acadoWorkspace.DyN[8]*acadoVariables.WN[104];
tmpDyN[9] = + acadoWorkspace.DyN[9]*acadoVariables.WN[117];
tmpDyN[10] = + acadoWorkspace.DyN[10]*acadoVariables.WN[130];
tmpDyN[11] = + acadoWorkspace.DyN[11]*acadoVariables.WN[143];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6] + acadoWorkspace.DyN[7]*tmpDyN[7] + acadoWorkspace.DyN[8]*tmpDyN[8] + acadoWorkspace.DyN[9]*tmpDyN[9] + acadoWorkspace.DyN[10]*tmpDyN[10] + acadoWorkspace.DyN[11]*tmpDyN[11];

objVal *= 0.5;
return objVal;
}

