#pragma once

struct RegistrationInfo {
	// from home kinect
	static const int32_t nRGS_DX_START = 3145;
	static const int32_t nRGS_AX = 1681;
	static const int32_t nRGS_BX = -5;
	static const int32_t nRGS_CX = 1;
	static const int32_t nRGS_DX = 468;

	static const int32_t nRGS_DY_START = 6771;
	static const int32_t nRGS_AY = -8;
	static const int32_t nRGS_BY = 2647;
	static const int32_t nRGS_CY = 599;
	static const int32_t nRGS_DY = 11;

	static const int32_t nRGS_DX_BETA_START = 10062;
	static const int32_t nRGS_DY_BETA_START = 130801;

	static const int32_t nRGS_DX_BETA_INC = 170;
	static const int32_t nRGS_DY_BETA_INC = 136;
	static const int32_t nRGS_DXDX_START = 2095911;
	static const int32_t nRGS_DXDY_START = 682;
	static const int32_t nRGS_DYDX_START = 651;
	static const int32_t nRGS_DYDY_START = 2096067;

	static const int32_t nRGS_DXDXDX_START = 134215532;
	static const int32_t nRGS_DYDXDX_START = 134217276;
	static const int32_t nRGS_DXDXDY_START = 134217118;
	static const int32_t nRGS_DYDXDY_START = 134216919;
	static const int32_t nRGS_DYDYDX_START = 134217176;
	static const int32_t nRGS_DYDYDY_START = 134215010;
};

struct RegistrationPadInfo {
	static const uint16_t nStartLines = 0;
	static const uint16_t nEndLines = 0;
	static const uint16_t nCroppingLines = 0;
};

void setupRegisterDepth();
void registerDepth(uint16_t* input, uint16_t* output);