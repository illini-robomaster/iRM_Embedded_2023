
const int order = 14;
typedef struct{
	float raw_value;
	float xbuf[order];
	float ybuf[order];
	float filtered_value;
}Filter_t;

#define real32_T float

const real32_T NUM[14] = {
   0.005271533504,  0.02838603035,  0.09061922133,   0.2069149017,   0.3684999347,
     0.5323253274,   0.6369557977,   0.6369557977,   0.5323253274,   0.3684999347,
     0.2069149017,  0.09061922133,  0.02838603035, 0.005271533504
};

const real32_T DEN[14] = {
                1,  -0.8707374334,    2.497394085,   -1.265378714,    2.054560423,
    -0.5266657472,   0.7393397093, -0.04414281249,   0.1291515678,  0.01134876721,
    0.01109408587,  0.00160036434,0.0003546577937,2.654808304e-05
};

float Chebyshev50HzLPF(Filter_t *F)
{
	int i;
	for(i=order - 1; i>0; i--)
	{
		F->ybuf[i] = F->ybuf[i-1]; 
		F->xbuf[i] = F->xbuf[i-1];
	}
	F->xbuf[0] = F->raw_value;
	F->ybuf[0] = NUM[0] * F->xbuf[0];
	for(i=1;i<order;i++)
	{
		F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
	}
	F->filtered_value = F->ybuf[0];
	return F->filtered_value;
}
