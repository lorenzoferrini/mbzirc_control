#include <math.h>
#include <cstdlib>
#include <iostream>
//#include <eigen3/Eigen/Core>
#include <algorithm>
#include <vector>
#include <random>
#include <queue>

using namespace std;
//using namespace Eigen;

// #TODO: take the trajectory from ros, not form the function!
//for now I am building the bare skeleton

// #TODO: to be changed from 2 to 3 dimensions;
//Matrix<float, 2, Dynamic>pos;

vector<float> posx;
vector<float> posy;
vector<folat>timeStamps;
int nSamples;
#define ERR_TOL_COEFF 2
#define HALF_TIME_TOL 3 //HALF_TIME_TOL=3; %the tolerance with wich we consider that two points are "near"
#define WINDOW_SIZE 25 //the size of the "samples" window
float* err=NULL;
vector<int> periodIdxCandidates;

vector< vector<float> > mobileWinErrors;
vector<int> candidatesPerLap;

/* CONSTRUCTION OF THE TRAJECTORY
Dovra' essere costruita dinamicamente. Inoltre, 
bisogna decidere "quando" (dopo quanti campioni) far partire l'analisi
Per ora ci atteniamo al matlab.
magari l'analisi si può far partire come serivizio ros */

void buildTraj(float step=0.1, float endTime=30){
    float dimOtto=10; //la dimensione lungo l'asse x dell'8
    float deviazione=dimOtto/20;
    default_random_engine generator; //da c++ reference
    normal_distribution<double> distribution(0.0,dimOtto);

    float t=0;
    while(t<=endTime){
        posx.push_back(a*2*sin(t)/(sin(t)^2+1))+
            +distribution(generator);
        posy.push_back(a*2*sin(t)/(sin(t)^2+1)*cos(t)+
            +distribution(generator));

        timeStamps.push_back(t);
        t+=step;
    }
    nSamples=posx.size();
}

float distanceBetweenTwoWins(int ax, int bx, int ay, int by, int len){
    /*calcola la distanza tra due vettori;
    Siccome i vettori vengono "estratti" dai campionamenti in x ed y, basta prendere gli indici;
    Inoltre nei due casi in cui si chiama la funzione, la distanza non è tra a e b bensi' 
    tra (a-b) e (c-d), da qui la necessita di quattro indici */

    // #TODO: rifare con eigen(?)
    // #Todo: rifare con gli iterators?
    //assumiamo che tutte le dimensioni siano non fuori scala del vettore
    float sum=0.0;
    for(int i=0;i<len;i++){
        sum+=sqrt(  (posx[ax+i]-posx[bx+i])^2 +(posy[ay+i]-posy[by+i])^2 );
    }

}



void preliminarAnalysis(){
    err=new float[nSamples-WINDOW_SIZE];
    float minErr=MAXFLOAT; // #TODO: check TX2
    //float w0x[WINDOW_SIZE];
    //float w0y[WINDOW_SIZE];
    //float wx[WINDOW_SIZE];
    //float wy[WINDOW_SIZE];
    //vector<float>::iterator itx=posx.begin();
    //vector<float>::iterator ity=posy.begin();
    // #TODO: redo this for with eigen/iterators, to make it faster
    // #SUPEROTTIMIZZAZIONE
    /*for(int i=0; i<WINDOW_SIZE;i++){
        w0x[i]=posx[i];
        w0y[i]=posy[i];
        //itx++;
        //ity++;
        /*wx[i]=posx[i+1];
        wy[i]=posy[i+1];
        //iniziallizza le finestre mobili gia shiftate
        In realta' le finestre mobili non servono: 
        costruisco ad hoc la funzione per calcolare la distanza
        tra vettori basata già sugli intervalli*//*
    }*/

    //main for for the distances on mobile windows
    for (int i=1; i<nSamples-WINDOW_SIZE-1;i++){
        err[i]=distanceBetweenTwoWins(i,0,i,0,WINDOW_SIZE);
        if(err[i]<minErr)
            minErr=err[i];
    }

    //NOTE: THE FIRST ELEMENT OF ERR err[0] WILL STILL BE -42
    bool auxBeginFlag=true;
    for(int i=1; i<nSamples-WINDOW_SIZE;i++){
        if(!auxBeginFlag && err[i]<=minErr*ERR_TOL_COEFF)
            periodIdxCandidates.push_back(i);
        else
            auxBeginFlag=false;
            //in questo modo, faccio si che le prime finestre non vengano prese in considerazione
    }
    
}

int findNearestTemporalCorrespondant(int currIdx, float T, int cci2){
    int idxCorr=0;
    float min=abs(T-timeStamps[0]);

    for(int i=1; i<nSamples;i++){
        if(min>=abs(T-timeStamps[i])){
            min=abs(T-timeStamps[i]);
            idxCorr=i;
        }
    }
    
    return  idxCorr;
}

void mobileWindowDist(int iStart, int nLap, int cci,int winSize=WINDOW_SIZE){
    float T=timeStamps[iStart]; // /nGiro; //index start lap
    int currIdx=iStart; //current index
    int idxCorr; //index of the correspondant
    while(timeStamps[currIdx]<T*(nLap+1)/nLap && currIdx+winSize<nSamples){
        idxCorr=findNearestTemporalCorrespondant(currIdx,T);
        mobileWinErrors[cci].push_back(
            distanceBetweenTwoWins(currIdx,idxCorr,currIdx,idxCorr,winSize);
        );
        currIdx++;
    }

}

int main( int argc, const char* argv[] )
{
    buildTraj(); //posx,posy,nSamples,timestamps

    preliminarAnalysis(); //err, periodIdxCandidates
    
    int cci=0; //comulative dandiate index, for readability
    int cci2=0;
    int iLap=0;
    queue<int> candidatiGiro;
    vector<float> endResults; //timestamps di inizio giro per ogni giro, da qui il periodo è na crema
    float currAvg;
    float minAvg;
    int lapWinningCandidate;
    
    //assert: periodIdxCandidates è strettamente monotono e crescente
    while (cci<periodIdxCandidates.size())
    {
        iLap+=1;
        while(abs(periodIdxCandidates[cci2] - periodIdxCandidates[cci])<=2*HALF_TIME_TOL){
            candidatiGiro.push(periodIdxCandidates[cci2]);
            mobileWinErrors.push_back(vector<float>);
            cci2++;
        }//trova gli indici in periodIdxCand in "gruppi vicini" 
        candidatesPerLap.push_back(candidatiGiro.size());

        //minAvg=MAXFLOAT; //giusto per iniziallizzare
        while (!candidatiGiro.empty()){
            mobileWindowDist(candidatiGiro.front(),iLap,cci,WINDOW_SIZE)            
            
            //calculate the mean error
            currAvg=0;
            for(int i=0;i<mobileWinErrors[cci].size();i++)
                currAvg+=mobileWinErrors[cci][i];

            currAvg=currAvg/mobileWinErrors[cci].size();
            if(currAvg<=minAvg || cci==cci2-candidatiGiro.size())
                lapWinningCandidate=candidatiGiro.front();
            
            candidatiGiro.pop();            
            cci++;
        }
        endResults.push_back(lapWinningCandidate);
        cout<<"GIRO "<<iLap<<endl;
        for(int i=0;i<candidatesPerLap[iLap-1];i++){
            int candidatoInEsame=periodIdxCandidates[cci2-candidatesPerLap[iLap-1]+i];
            cout<<"cand. idx:"<<periodIdxCandidates[candidatoInEsame]<<
                <<" timestamp:"<<timeStamps[candidatoInEsame]<<
                <<" dall'ultimo:";
                if(iLap==1)
                    cout<<timeStamps[candidatoInEsame];
                else
                    cout<<timeStamps[candidatoInEsame]-timeStamps[endResults[iLap-2]];
                cout<<endl;
        }
    }
    

    return 0;
}