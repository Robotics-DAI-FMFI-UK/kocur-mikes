#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "mikes_logs.h"
#include "rfid_sensor.h"
#include "mcl.h"


pthread_mutex_t range_mcl_lock;

hypo_t hypo[2][HYPO_COUNT];
int activeHypo = 0; 

double last_traveled = 0;

static rfid_data_type rfid_data;


double generateGaussianNoise(double mu, double sigma)
{
	const double epsilon = 0.000000001;
	const double two_pi = 2.0*3.14159265358979323846;

	static double z0, z1;
	static int generate;
	generate = 1-generate;

	if (!generate)
	   return z1 * sigma + mu;

	double u1, u2;
	do
	 {
	   u1 = rand() * (1.0 / RAND_MAX);
	   u2 = rand() * (1.0 / RAND_MAX);
	 }
	while ( u1 <= epsilon );

	z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
	return z0 * sigma + mu;
}



int init_mcl(){
    pthread_mutex_init(&range_mcl_lock, 0);
    time_t t;
    srand((unsigned) time(&t));
    for(int i = 0; i< HYPO_COUNT; i++){
            hypo[0][i].x = hypo[1][i].x = rand() % 280;
            hypo[0][i].y = hypo[1][i].y = rand() % 280;
            hypo[0][i].alpha = hypo[1][i].alpha = rand() % 360;
            hypo[0][i].w = hypo[1][i].w = 0.3;

            mikes_log_val(ML_INFO, "hypo id: ", i);
            mikes_log_double2(ML_INFO, "hypo pos: ", hypo[0][i].x,hypo[0][i].y);
            mikes_log_double2(ML_INFO, "hypo v&a: ", hypo[0][i].w,hypo[0][i].alpha);

    }
    return 0;
}

void get_mcl_data(hypo_t *buffer)
{
    pthread_mutex_lock(&range_mcl_lock);
    memcpy(buffer, hypo[activeHypo], sizeof(hypo_t) * HYPO_COUNT);
    pthread_mutex_unlock(&range_mcl_lock);
}

double normAlpha(double alpha){
	if(alpha < 0){
		while(alpha < 0)
			alpha += 360;
	}
	else
		while(alpha >= 360)
			alpha -= 360;
	return alpha;		
}

double getp( double x, double y){
    return fmax(-0.001*abs(x*x + 4*y*y) + 0.9, 0);
}

int mcl_update(double traveled, int heading){
    mikes_log_double(ML_INFO, "MCL New data - traveled:", traveled);
    mikes_log_val(ML_INFO, "MCL New data - heading:", heading);

    get_rfid_data(&rfid_data);

    pthread_mutex_lock(&range_mcl_lock);
    activeHypo = 1-activeHypo;
 
    for(int i = 0; i< HYPO_COUNT; i++){
            mikes_log_val(ML_DEBUG, "hypo id premove: ", i);
            mikes_log_val2(ML_DEBUG, "hypo pos: ", hypo[1-activeHypo][i].x,hypo[1-activeHypo][i].y);
            mikes_log_val2(ML_DEBUG, "hypo v&a: ", hypo[1-activeHypo][i].w*100,hypo[1-activeHypo][i].alpha);
    }

   
    for(int i = 0; i< HYPO_COUNT; i++){
        hypo[1-activeHypo][i].x += traveled * cos(normAlpha(hypo[1-activeHypo][i].alpha+heading)*M_PI/180.0);
        hypo[1-activeHypo][i].y -= traveled * sin(normAlpha(hypo[1-activeHypo][i].alpha+heading)*M_PI/180.0);
        hypo[1-activeHypo][i].alpha = normAlpha(hypo[1-activeHypo][i].alpha + heading);


        if((hypo[1-activeHypo][i].x < 0 )||(280 <hypo[1-activeHypo][i].x)||(hypo[1-activeHypo][i].y < 0)||(280 < hypo[1-activeHypo][i].y)){
          hypo[1-activeHypo][i].w = 0;
          continue;
        }

        //sensor position
        double possx = hypo[1-activeHypo][i].x + cos(hypo[1-activeHypo][i].alpha*M_PI/180.0) * 22;
        double possy = hypo[1-activeHypo][i].y - sin(hypo[1-activeHypo][i].alpha*M_PI/180.0) * 22;
        //tag position
        double minposx;
        double minposy;

        if(rfid_data.ntags == 0){

            // middle of square
            double possqx = (possx + cos(hypo[1-activeHypo][i].alpha*M_PI/180.0) * 20);
            double possqy = (possy - sin(hypo[1-activeHypo][i].alpha*M_PI/180.0) * 20);
            // id of middle tag
            double idmidx = (possx + cos(hypo[1-activeHypo][i].alpha*M_PI/180.0) * 20) / 40 + 1;
            double idmidy = (possy - sin(hypo[1-activeHypo][i].alpha*M_PI/180.0) * 20) / 40 + 1;
            // vector of robot direction
            double vec1x = possqx - possx;
            double vec1y = possqy - possy;

            double minval = 50;

            for(int i = -1; i<= 1; i++)
              for(int j = -1; i<= j; i++){
                  // position of tag
                 double postagx = (idmidx-1+i)*40;
                 double postagy = (idmidy-1+j)*40;
                 // vector of tag
                 double vec2x = postagx - possx;
                 double vec2y = postagy - possy;
                 double dotProd = (vec1x*vec2x)+(vec1y*vec2y);

                 if((dotProd >= 0) && (( (postagx-possx)*(postagx-possx) + (postagy-possy)*(postagy-possy) ) < minval)){
                     minval = (postagx-possx)*(postagx-possx) + (postagy-possy)*(postagy-possy);
                     minposx = postagx;
                     minposy = postagy;
                 }
              }
            hypo[1-activeHypo][i].w = (1-getp(possx-minposx, possy-minposy)) * hypo[1-activeHypo][i].w / ((1600-636.173)/1600);  
        }else{ // dufame, ze uvidi len jeden
            minposx = (rfid_data.x[0]-1)*40;
            minposy = (rfid_data.y[0]-1)*40;
            hypo[1-activeHypo][i].w =    getp(possx-minposx, possy-minposy)  * hypo[1-activeHypo][i].w /       (636.173 /1600);  			
		}
		hypo[1-activeHypo][i].w = fmin(hypo[1-activeHypo][i].w, 1);
		
    }

    for(int i = 0; i< HYPO_COUNT; i++){
            mikes_log_val(ML_DEBUG, "hypo id postmove: ", i);
            mikes_log_val2(ML_DEBUG, "hypo pos: ", hypo[1-activeHypo][i].x,hypo[1-activeHypo][i].y);
            mikes_log_val2(ML_DEBUG, "hypo v&a: ", hypo[1-activeHypo][i].w*100,hypo[1-activeHypo][i].alpha);
    }
	
    double cumP[HYPO_COUNT];
    double last = 0;
    for(int i = 0; i<= HYPO_COUNT; i++){
		last += hypo[1-activeHypo][i].w;
		cumP[i] = last;
	}
    int i;
    for(i = 0; i < HYPO_COUNT*0.9; i++){
		double next = (double)rand() / (double)RAND_MAX * last;
		for(int j = 0; j< HYPO_COUNT; j++){
			if( next <= cumP[j]){
				hypo[activeHypo][i].x = hypo[1-activeHypo][j].x + generateGaussianNoise(0, 0.03*traveled);
				hypo[activeHypo][i].y = hypo[1-activeHypo][j].y + generateGaussianNoise(0, 0.03*traveled);
				hypo[activeHypo][i].alpha = normAlpha(hypo[1-activeHypo][j].alpha + generateGaussianNoise(0, heading*0.05));
				hypo[activeHypo][i].w = hypo[1-activeHypo][j].w;
				break;
			}	
		}	
	}
   for(; i< HYPO_COUNT; i++){
            hypo[0][i].x = hypo[1][i].x = rand() % 280;
            hypo[0][i].y = hypo[1][i].y = rand() % 280;
            hypo[0][i].alpha = hypo[1][i].alpha = rand() % 360;
            hypo[0][i].w = hypo[1][i].w = 0.01;

    }	

    pthread_mutex_unlock(&range_mcl_lock);

    return 0;
}
