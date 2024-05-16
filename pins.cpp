// Automatically generated C++ file on Mon May  6 19:36:34 2024
//
// To build with Digital Mars C++ Compiler: 
//
//    dmc -mn -WD pins.cpp kernel32.lib

union uData
{
   bool b;
   char c;
   unsigned char uc;
   short s;
   unsigned short us;
   int i;
   unsigned int ui;
   float f;
   double d;
   long long int i64;
   unsigned long long int ui64;
   char *str;
   unsigned char *bytes;
};

// int DllMain() must exist and return 1 for a process to load the .DLL
// See https://docs.microsoft.com/en-us/windows/win32/dlls/dllmain for more information.
int __stdcall DllMain(void *module, unsigned int reason, void *reserved) { return 1; }

// #undef pin names lest they collide with names in any header file(s) you might include.
#undef v_ref
#undef vline_sense
#undef Vo_sense
#undef IL_sense
#undef duty
#undef Shigh
#undef Slow

extern "C" __declspec(dllexport) void pins(void **opaque, double t, union uData *data)
{
   double  v_ref        = data[ 0].d; // input
   double  vline_sense  = data[ 1].d; // input
   double  Vo_sense     = data[ 2].d; // input
   double  IL_sense     = data[ 3].d; // input
   double  fs           = data[ 4].d; // input parameter
   double  fclk         = data[ 5].d; // input parameter
   double  kpi          = data[ 6].d; // input parameter
   double  kii          = data[ 7].d; // input parameter
   double  kpv          = data[ 8].d; // input parameter
   double  kiv          = data[ 9].d; // input parameter
   double  tdead        = data[10].d; // input parameter
   int     carrier_mode = data[11].i; // input parameter
   int     sample_mode  = data[12].i; // input parameter
   double &duty         = data[13].d; // output
   double &Shigh        = data[14].d; // output
   double &Slow         = data[15].d; // output

// Implement module evaluation code here:

}
