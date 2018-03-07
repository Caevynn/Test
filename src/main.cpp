#include <iostream>
#include <cmath>

#include "global/types.h"

using namespace std;

vector<double> x = {10, 50, 20, -10, 60};//, -50};
vector<double> y = {50, -10, 30, 60, -20};//, 20};

InstanceData ID;
SolutionData SD;

Point Depot = Point();

void LoadInstanceData(void)
{
  Address adr = Address();

  for(int i = 0; i < x.size(); ++i)
  {
    adr.x = x[i];
    adr.y = y[i];
    adr.code = 8000 + i;
    ID.vAdr.push_back(adr);
  }
}

void InitializeSolution(InstanceData& id, SolutionData* sd)
{
  Route rt = Route();
  OprPlan OP = OprPlan();
  int nAdr = id.vAdr.size();
  int nStops = 0;

  for(int i = 0; i < nAdr; ++i)
  {
    OP.iAdr = i;
    OP.iPre = i-1;
    OP.iNext = i+1;
    sd->vOprPlan.push_back(OP);

    rt.vStops.push_back(i);
  }
  sd->vOprPlan[nAdr-1].iNext = -1;

  rt.iFirst = 0;
  rt.iLast = nAdr-1;

  sd->vRoute.push_back(rt);
}

double CalculateDistance(Point& a, Point& b)
{
  return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2));
}

double CalculateRouteDistance(Route& rt)
{
  double d = 0;
  int i = rt.iFirst;
  int next = -1;

  d += CalculateDistance(Depot,ID.vAdr[SD.vOprPlan[i].iAdr]);

  for(; i != rt.iLast; i = next)
  {
    next = SD.vOprPlan[i].iNext;

    if(next < 0) cout << "ERROR!" << endl;

    d += CalculateDistance(ID.vAdr[SD.vOprPlan[i].iAdr],
                           ID.vAdr[SD.vOprPlan[next].iAdr]);
  }

  d += CalculateDistance(Depot,ID.vAdr[SD.vOprPlan[i].iAdr]);

  return d;
}

double OptimizeRoute(Route& rt)
{
  double dOpt = rt.Distance;
  int nStops = rt.vStops.size();
  int iter = nStops - 2;

  vector<int> vOrder;
  for(int i = rt.iFirst; i != rt.iLast; i = SD.vOprPlan[i].iNext)
  {
    vOrder.push_back(i);
  }
  vOrder.push_back(rt.iLast);
  cout << vOrder.size() << endl;

  cout << nStops << " " << iter << endl;

  for(int i = 0; i < nStops - 1; ++i)
  {
    if(i > 1) --iter;
    for(int j = 0; j < iter; ++j)
    {

    }
  }
}

int main()
{
  LoadInstanceData();
  InitializeSolution(ID,&SD);

  cout << "Distance of Tour: " << CalculateRouteDistance(SD.vRoute[0]) << endl;

  OptimizeRoute(SD.vRoute[0]);

  return 0;
}
