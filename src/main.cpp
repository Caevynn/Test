#include <iostream>
#include <cmath>
#include <random>

#include "types.h"

using namespace std;

#define N_ROUTES 2
#define N_ADRESSES N_ROUTES*100

/*vector<double> x = {10, 50, 20, -10, 60, -50};
vector<double> y = {50, -10, 30, 60, -20, 20};*/

vector<double> x = {10,0,20,0,20,10,20};
vector<double> y = {0,10,0,20,20,20,10};

InstanceData ID;
SolutionData SD;

Point Depot = Point();
int iDepot;

void LoadInstanceData(void)
{
  Address adr = Address();

  int i = 0;
  for(; i < x.size(); ++i)
  {
    adr.x = x[i];
    adr.y = y[i];
    adr.code = 8000 + i;
    ID.vAdr.push_back(adr);
  }
  adr.x = 0;
  adr.y = 0;
  adr.code = 8000 + i;
  ID.vAdr.push_back(adr);
  iDepot = i;
}

void LoadInstanceDataRandom(void)
{
  random_device rd;
  mt19937 gen(rd());
  uniform_int_distribution<> dis(1,1000);

  Address adr = Address();

  int i = 0;
  for(; i < N_ADRESSES; ++i)
  {
    adr.x = dis(gen);
    adr.y = dis(gen);
    adr.code = 8000 + i;
    ID.vAdr.push_back(adr);
  }
  adr.x = 0;
  adr.y = 0;
  adr.code = 8000 + i;
  ID.vAdr.push_back(adr);
  iDepot = i;
}

void InitializeSolution(void)
{
  Route rt = Route();
  OprPlan op = OprPlan();
  int nAdr = ID.vAdr.size()/N_ROUTES;

  for(int j = 0; j < N_ROUTES; ++j)
  {
    int offset = nAdr*j;
    int nStops = 0;
    for(int i = 0; i < nAdr; ++i)
    {
      op.iAdr = i + offset;
      op.iPre = i-1 + offset;
      op.iNext = i+1 + offset;
      SD.vOprPlan.push_back(op);
      ++nStops;
    }
    SD.vOprPlan[nAdr-1].iNext = -1;

    rt.iFirst = offset;
    rt.iLast = nAdr-1 + offset;
    rt.nStops = nStops;
    SD.vRoute.push_back(rt);
  }

  op.iAdr = iDepot;
  op.iPre = -1;
  op.iNext = -1;
  SD.vOprPlan.push_back(op);
}

void PrintSolutionVector(vector<int> vOpt)
{
  int nStops = vOpt.size();

  for(int i = 0; i < nStops; ++i)
    cout << vOpt[i] << " ";

  cout << endl;
}

double CalculateDistancePoints(Point& a, Point& b)
{
  return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2));
}

double CalculateRouteDistanceVector(vector<int> vStops)
{
  double d = 0;
  int nStops = vStops.size()-1;

  d += CalculateDistancePoints(ID.vAdr[iDepot],
							   ID.vAdr[SD.vOprPlan[vStops[0]].iAdr]);
  for(int i = 0; i < nStops; ++i)
  {
    //cout << vStops[i] << " " << vStops[i+1] << endl;
    d += CalculateDistancePoints(ID.vAdr[SD.vOprPlan[vStops[i]].iAdr],
                                 ID.vAdr[SD.vOprPlan[vStops[i+1]].iAdr]);
  }
  d += CalculateDistancePoints(ID.vAdr[iDepot],
							   ID.vAdr[SD.vOprPlan[vStops[nStops]].iAdr]);

  return d;
}

double CalculateRouteDistance(Route& rt)
{
  double d = 0;
  int i = rt.iFirst;
  int next;

  d += CalculateDistancePoints(ID.vAdr[iDepot],
							   ID.vAdr[SD.vOprPlan[i].iAdr]);
  for(; i != rt.iLast; i = next)
  {
    next = SD.vOprPlan[i].iNext;
    d += CalculateDistancePoints(ID.vAdr[SD.vOprPlan[i].iAdr],
                                 ID.vAdr[SD.vOprPlan[next].iAdr]);
  }
  d += CalculateDistancePoints(ID.vAdr[iDepot],
							   ID.vAdr[SD.vOprPlan[i].iAdr]);

  return d;
}

void CreateVectorFromRoute(Route& rt, vector<int>* vOpt)
{
  vOpt->clear();
  for(int i = rt.iFirst; i != rt.iLast; i = SD.vOprPlan[i].iNext)
  {
    vOpt->push_back(i);
  }
  vOpt->push_back(rt.iLast);
}

void SwapTwoOptVector(vector<int>& vOpt, vector<int>* vNew, int i, int j)
{
  int k = i+j+1;
  *vNew = vOpt;

  for(int l = i; l < k; ++l)
  {
    (*vNew)[l] = vOpt[k];
    (*vNew)[k] = vOpt[l];
    --k;
  }
}

int UpdateRouteFromVector(vector<int>& vOpt, Route& rt)
{
  rt.iFirst = vOpt[0];
  SD.vOprPlan[rt.iFirst].iPre = -1;
  SD.vOprPlan[rt.iFirst].iNext = vOpt[1];

  for(int i = 1; i < rt.nStops-1; ++i)
  {
    SD.vOprPlan[vOpt[i]].iNext = vOpt[i+1];
    SD.vOprPlan[vOpt[i]].iPre = vOpt[i-1];
  }

  rt.iLast = vOpt[rt.nStops-1];
  SD.vOprPlan[rt.iLast].iPre = vOpt[rt.nStops-2];
  SD.vOprPlan[rt.iLast].iNext = -1;

  return 1;
}

void TwoOptIntraRoute(Route& rt, int nIter)
{
  vector<int> vOpt, vNew;

  double dOpt, dNew;

  int cnt, improved;

  for(int o = 0; o < nIter; ++o)
  {
    CreateVectorFromRoute(rt,&vOpt);
    dOpt = CalculateRouteDistanceVector(vOpt);

    cnt = 0;
    improved = 0;

    int iter = rt.nStops - 2;
    for(int i = 0; i < rt.nStops - 1; ++i)
    {
      if(i > 1) --iter;
      for(int j = 0; j < iter; ++j)
      {
        SwapTwoOptVector(vOpt,&vNew,i,j);
        dNew = CalculateRouteDistanceVector(vNew);

        if(dNew < dOpt)
        {
          improved = 1;
          vOpt = vNew;
          dOpt = dNew;
        }

        ++cnt;
      }
    }

    //cout << cnt << endl;
    if(!UpdateRouteFromVector(vOpt,rt) || !improved)
      return;
  }
}

void CombineTwoVectors(vector<int>& vRt1,
					   vector<int>& vRt2,
					   vector<int>* vNew,
					   int i, int j)
{
  int k = 0;
  int offset;

  // Combine first route
  offset = k;
  for(; k < offset + i; ++k)
    (*vNew)[k] = vRt1[k - offset];

  offset = k;
  for(; k < offset + vRt2.size() - j; ++k)
    (*vNew)[k] = vRt2[j + k - offset];

  // Include depot
  (*vNew)[k++] = iDepot;

  // Combine second route
  offset = k;
  for(;k < offset + j; ++k)
    (*vNew)[k] = vRt2[k - offset];

  offset = k;
  for(; k < offset + vRt1.size() - i; ++k)
    (*vNew)[k] = vRt1[i + k - offset];
}

int UpdateRoutesFromVector(vector<int>& vOpt, Route& rt1, Route& rt2)
{
  int nStops = 0;
  int i = 0;

  // Create first route
  rt1.iFirst = vOpt[i];
  SD.vOprPlan[vOpt[i]].iPre = -1;
  SD.vOprPlan[vOpt[i]].iNext = vOpt[i+1];
  ++nStops;

  ++i;
  for(; vOpt[i+1] != iDepot; ++i)
  {
    SD.vOprPlan[vOpt[i]].iPre = vOpt[i-1];
    SD.vOprPlan[vOpt[i]].iNext = vOpt[i+1];
    ++nStops;
  }

  rt1.iLast = vOpt[i];
  SD.vOprPlan[vOpt[i]].iPre = vOpt[i-1];
  SD.vOprPlan[vOpt[i]].iNext = -1;
  ++nStops;

  rt1.nStops = nStops;

  // Create second route
  i += 2;
  if(i >= vOpt.size() - 1)
    return 0;

  nStops = 0;
  rt2.iFirst = vOpt[i];
  SD.vOprPlan[vOpt[i]].iPre = -1;
  SD.vOprPlan[vOpt[i]].iNext = vOpt[i+1];
  ++nStops;

  ++i;
  for(; i < vOpt.size() - 1; ++i)
  {
    SD.vOprPlan[vOpt[i]].iPre = vOpt[i-1];
    SD.vOprPlan[vOpt[i]].iNext = vOpt[i+1];
    ++nStops;
  }

  rt2.iLast = vOpt[i];
  SD.vOprPlan[vOpt[i]].iPre = vOpt[i-1];
  SD.vOprPlan[vOpt[i]].iNext = -1;
  ++nStops;

  rt2.nStops = nStops;

  return 1;
}

void TwoOptInterRoute(Route& rt1, Route& rt2, int nIter)
{
  vector<int> vRt1, vRt2;
  vector<int> vOpt(rt1.nStops + rt2.nStops + 1);
  vector<int> vNew(rt1.nStops + rt2.nStops + 1);

  double dOpt, dNew;

  int cnt, improved;

  for(int o = 0; o < nIter; ++o)
  {
    CreateVectorFromRoute(rt1, &vRt1);
    CreateVectorFromRoute(rt2, &vRt2);

    CombineTwoVectors(vRt1, vRt2, &vOpt, 0, 0);
    dOpt = CalculateRouteDistanceVector(vOpt);

    cnt = 0;
    improved = 0;

    for(int i = 1; i < rt1.nStops; ++i)
    {
      for(int j = 1; j < rt2.nStops; ++j)
      {
        CombineTwoVectors(vRt1, vRt2, &vNew, i, j);
        dNew = CalculateRouteDistanceVector(vNew);

        if(dNew < dOpt)
        {
          improved = 1;
          vOpt = vNew;
          dOpt = dNew;
        }

        ++cnt;
      }
    }

    //cout << cnt << endl;
    if(!UpdateRoutesFromVector(vOpt,rt1,rt2) || !improved)
      return;
  }
}

int main()
{
  LoadInstanceDataRandom();
  InitializeSolution();

  cout << "Tour 1: " << CalculateRouteDistance(SD.vRoute[0]) << ", " << SD.vRoute[0].nStops << endl;
  cout << "Tour 2: " << CalculateRouteDistance(SD.vRoute[1]) << ", " << SD.vRoute[1].nStops << endl;

  for(int i = 0; i < 2; ++i)
  {
    TwoOptIntraRoute(SD.vRoute[0],2);
    TwoOptIntraRoute(SD.vRoute[1],2);
    cout << "-";

    //cout << endl << "Tour 1: " << CalculateRouteDistance(SD.vRoute[0]) << ", " << SD.vRoute[0].nStops << endl;
    //cout << "Tour 2: " << CalculateRouteDistance(SD.vRoute[1]) << ", " << SD.vRoute[1].nStops << endl;

    TwoOptInterRoute(SD.vRoute[0],SD.vRoute[1],4);

    //cout << endl << "Tour 1: " << CalculateRouteDistance(SD.vRoute[0]) << ", " << SD.vRoute[0].nStops << endl;
    //cout << "Tour 2: " << CalculateRouteDistance(SD.vRoute[1]) << ", " << SD.vRoute[1].nStops << endl;
    cout << ".";
  }

  cout << endl;
  cout << "Tour 1: " << CalculateRouteDistance(SD.vRoute[0]) << ", " << SD.vRoute[0].nStops << endl;
  cout << "Tour 2: " << CalculateRouteDistance(SD.vRoute[1]) << ", " << SD.vRoute[1].nStops << endl;

  return 0;
}
