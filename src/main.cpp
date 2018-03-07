#include <iostream>
#include <cmath>

#include "global/types.h"

using namespace std;

#define NO_2OPT_ITERATIONS 10

/*vector<double> x = {10, 50, 20, -10, 60, -50};
vector<double> y = {50, -10, 30, 60, -20, 20};*/

vector<double> x = {10,0,20,0,20,10,20};
vector<double> y = {0,10,0,20,20,20,10};

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
  OprPlan op = OprPlan();
  int nAdr = id.vAdr.size();
  int nStops = 0;

  for(int i = 0; i < nAdr; ++i)
  {
    op.iAdr = i;
    op.iPre = i-1;
    op.iNext = i+1;
    sd->vOprPlan.push_back(op);
    ++nStops;
  }
  sd->vOprPlan[nAdr-1].iNext = -1;

  rt.iFirst = 0;
  rt.iLast = nAdr-1;
  rt.nStops = nStops;
  sd->vRoute.push_back(rt);
}

void PrintSolutionVector(vector<int> vOrder)
{
  int nStops = vOrder.size();

  for(int i = 0; i < nStops; ++i)
    cout << vOrder[i] << " ";

  cout << endl;
}

double CalculateDistance(Point& a, Point& b)
{
  return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2));
}

double CalculateRouteDistanceVector(vector<int> vStops)
{
  double d = 0;
  int nStops = vStops.size()-1;

  d += CalculateDistance(Depot,ID.vAdr[SD.vOprPlan[vStops[0]].iAdr]);
  for(int i = 0; i < nStops; ++i)
  {
    d += CalculateDistance(ID.vAdr[SD.vOprPlan[vStops[i]].iAdr],
                           ID.vAdr[SD.vOprPlan[vStops[i+1]].iAdr]);
  }
  d += CalculateDistance(Depot,ID.vAdr[SD.vOprPlan[vStops[nStops]].iAdr]);

  return d;
}

double CalculateRouteDistance(Route& rt)
{
  double d = 0;
  int i = rt.iFirst;
  int next;

  d += CalculateDistance(Depot,ID.vAdr[SD.vOprPlan[i].iAdr]);
  for(; i != rt.iLast; i = next)
  {
    next = SD.vOprPlan[i].iNext;
    d += CalculateDistance(ID.vAdr[SD.vOprPlan[i].iAdr],
                           ID.vAdr[SD.vOprPlan[next].iAdr]);
  }
  d += CalculateDistance(Depot,ID.vAdr[SD.vOprPlan[i].iAdr]);

  return d;
}

void CreateVectorFromRoute(Route& rt, vector<int>* vOrder)
{
  for(int i = rt.iFirst; i != rt.iLast; i = SD.vOprPlan[i].iNext)
  {
    vOrder->push_back(i);
  }
  vOrder->push_back(rt.iLast);
}

void SwapTwoOptVector(vector<int>& vOrder, vector<int>* vCopy, int i, int j)
{
  int k = i+j+1;
  *vCopy = vOrder;

  for(int l = i; l < k; ++l)
  {
    (*vCopy)[l] = vOrder[k];
    (*vCopy)[k] = vOrder[l];
    --k;
  }
}

void UpdateRouteFromVector(vector<int>& vOrder, Route& rt)
{
  rt.iFirst = vOrder[0];
  SD.vOprPlan[rt.iFirst].iPre = -1;
  SD.vOprPlan[rt.iFirst].iNext = vOrder[1];

  for(int i = 1; i < rt.nStops-1; ++i)
  {
    SD.vOprPlan[vOrder[i]].iNext = vOrder[i+1];
    SD.vOprPlan[vOrder[i]].iPre = vOrder[i-1];
  }

  rt.iLast = vOrder[rt.nStops-1];
  SD.vOprPlan[rt.iLast].iPre = vOrder[rt.nStops-2];
  SD.vOprPlan[rt.iLast].iNext = -1;
}

void OptimizeRoute(Route& rt)
{
  vector<int> vOrder, vCopy;

  CreateVectorFromRoute(rt,&vOrder);
  double dOpt = CalculateRouteDistanceVector(vOrder);

  for(int o = 0; o < NO_2OPT_ITERATIONS; ++o)
  {
    int iter = rt.nStops - 2;
    for(int i = 0; i < rt.nStops - 1; ++i)
    {
      if(i > 1) --iter;
      for(int j = 0; j < iter; ++j)
      {
        SwapTwoOpt(vOrder,&vCopy,i,j);
        double dNew = CalculateRouteDistanceVector(vCopy);

        if(dNew < dOpt)
        {
          vOrder = vCopy;
          dOpt = dNew;
        }
      }
    }
  }
  UpdateRouteFromVector(vOrder,rt);

  //PrintSolutionVector(vOrder);
}

int main()
{
  LoadInstanceData();
  InitializeSolution(ID,&SD);

  cout << "Distance of Tour: " << CalculateRouteDistance(SD.vRoute[0]) << endl;

  OptimizeRoute(SD.vRoute[0]);

  cout << "Distance of Tour: " << CalculateRouteDistance(SD.vRoute[0]) << endl;

  return 0;
}
