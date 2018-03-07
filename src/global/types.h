#include <vector>

using namespace std;

struct Point
{
	double x;
	double y;

	Point()
	{
		x = 0;
		y = 0;
	}
};

struct Address : Point
{
	int code;

	Address()
	{
		code = -1;
	}
};

struct OprPlan
{
  int iPre, iNext, iAdr;
  double dPre, dNext;

  OprPlan()
  {
    iPre = -1;
    iNext = -1;
    iAdr = -1;
  }
};

struct Route
{
  int iFirst, iLast;
  int nStops;
  vector<int> vStops;
  double Distance;

  Route()
  {
    iFirst = -1;
    iLast = -1;
    Distance = -1;
  }
};

struct SolutionData
{
  vector<OprPlan> vOprPlan;
  vector<Route> vRoute;
};

struct InstanceData
{
  vector<Address> vAdr;
};
