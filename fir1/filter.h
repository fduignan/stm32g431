#include <stdint.h>
// All values in the b vector were scaled up by this factor
#define SCALE_FACTOR 4096
#define FILTER_LENGTH (sizeof(b)/sizeof(float))
float b[]={
-7.777325,
-11.723946,
-15.332439,
-18.432192,
-20.866825,
-22.501396,
-23.228988,
-22.976428,
-21.708830,
-19.432765,
-16.197846,
-12.096622,
-7.262708,
-1.867158,
3.886840,
9.770812,
15.539164,
20.938405,
25.717110,
29.636249,
32.479504,
34.063173,
34.245262,
32.933375,
30.091076,
25.742402,
19.974302,
12.936822,
4.840954,
-4.045856,
-13.406461,
-22.882867,
-32.086916,
-40.612499,
-48.048886,
-53.994746,
-58.072374,
-59.941633,
-59.313096,
-55.959928,
-49.728039,
-40.544118,
-28.421227,
-13.461681,
4.142927,
24.114627,
46.094866,
69.653050,
94.297604,
119.489214,
144.655863,
169.209162,
192.561477,
214.143277,
233.420150,
249.908911,
263.192272,
272.931584,
278.877224,
280.876271,
278.877224,
272.931584,
263.192272,
249.908911,
233.420150,
214.143277,
192.561477,
169.209162,
144.655863,
119.489214,
94.297604,
69.653050,
46.094866,
24.114627,
4.142927,
-13.461681,
-28.421227,
-40.544118,
-49.728039,
-55.959928,
-59.313096,
-59.941633,
-58.072374,
-53.994746,
-48.048886,
-40.612499,
-32.086916,
-22.882867,
-13.406461,
-4.045856,
4.840954,
12.936822,
19.974302,
25.742402,
30.091076,
32.933375,
34.245262,
34.063173,
32.479504,
29.636249,
25.717110,
20.938405,
15.539164,
9.770812,
3.886840,
-1.867158,
-7.262708,
-12.096622,
-16.197846,
-19.432765,
-21.708830,
-22.976428,
-23.228988,
-22.501396,
-20.866825,
-18.432192,
-15.332439,
-11.723946,
-7.777325
};