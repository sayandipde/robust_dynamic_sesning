#include <Eigen/Eigen>
#include "config.hpp"

using namespace std;
using namespace Eigen;

// constructor
lateralController50_bev::lateralController50_bev() {
    // initialize member variables 
    // m_d   = 0.135L;   // 2*d = distance between left and right wheels
    // m_l   = 0.42L;    // l = distance between front and read wheels 
    // m_vx  = 2.2L;     // vehicle longitudinal speed is 2.2m/s to simulate ~80km/h in reality
    // m_LL  = 0.55L;    // look-ahead distance is 0.25s*vx
    // m_l_r = 0.21L;    // distance from CG to rear axle (m)
    m_d   = 1.628L;   // 2*d = distance between left and right wheels  
    m_l   = 2.995L;    // l = distance between front and rear wheels 
    m_vx  = 13.88889L;     // vehicle longitudinal speed is 5.55556m/s (20 km/hr)
    m_LL  = 5.5L;    // look-ahead distance is 0.25s*vx
    m_l_r = 1.2975L;    // distance from CG to rear axle (m)
    m_z1    = 0.0L;         // z1 is vy
    m_z2    = 0.0L;         // z2 is yaw rate
    m_z3    = 0.0L;         // z3 is yL
    m_z4    = 0.0L;         // z4 is epsilon_L
    m_z5    = 0.0L;         // z5 is curvature at lookahead distance KL (which is K_ref of CoG)
    fill(m_input.begin(), m_input.end(), 0.0L); // m_input is the input of the last sampling period.

    m_desired_steering_angle = 0.0L;
    m_steering_angle_left    = 0.0L;
    m_steering_angle_right   = 0.0L;

    // controller design time parameters
    // Eigen::Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
	///// VALUES ARE CONSIDERING ACTUATE DELAY //////////
/////////////////////////////////////////////////////////////// v0 ////////////////////////////////////////////////////////////////////////////
    m_phi_aug[0] <<   
0.74301174761195976,   -0.42226330801771633,   0,   0,   0,   1.5559451883362971,
     -0.014744435174581204,   0.80835641187682827,   0,   0,   0,   0.99557646247979037,
     -0.028656443027834801,   -0.17311895085723894,   1,   0.4861111111111111,   0.1181520061728395,   -0.13295226750854111,
     0.00028163632370048518,   -0.031501232385698755,   0,   1,   0.4861111111111111,   -0.018140508169899046,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;
    
    // q=7.5
    m_K2c[0] <<
-0.019350694520484407,   -0.1791061094978682,   0.18839365259349825,   1.1185873698088256,   -2.5109916471373994e-14,   -0.23033812319525854; // Q = 7.5
//-0.28614679486699318,   -0.28802685785999194,   -1.0593314709112267,   0.080818517814108204,   -0.15246689570600142; // Q = 2.5
    

    m_T[0] <<
-1.6084139228822458e-16,   2.9089144287786084e-16,   -2.7998436902265667e-15,   2.2676305277968822e-14,   1,   0,
     -0.002294781655814837,   0.0041097540687726024,   -0.13134140664343083,   0.99132602043907403,   -2.3175905639050143e-14,   0,
     0.35591111358533528,   -0.64710730117894388,   -0.66883193543184716,   -0.085107356520098015,   0,   0,
     -0.40930511396356506,   0.54062533347598662,   -0.72818957046292188,   -0.099667055521254128,   0,   0,
     0.84011388328420789,   0.5375495321236694,   -0.071785977157377764,   -0.0097947491194457417,   0,   0,
     0,   0,   0,   0,   0,   1;
  

    m_Gamma_aug[0] <<
0,
     0,
     -0,
     -0,
     0,
     1;
	 
/////////////////////////////////////////////////////////////// v1 ////////////////////////////////////////////////////////////////////////////
    m_phi_aug[1] <<   
0.87929873026351646,   -0.20962126769509487,   0,   0,   0,   0.79193149253770323,
     -0.0073194784724563006,   0.9117373321979696,   0,   0,   0,   0.45701690410326046,
     -0.013755613355515913,   -0.078674049008538458,   1,   0.20833333333333334,   0.021701388888888888,   -0.025635600870535839,
     5.6983253816493898e-05,   -0.014325591637869291,   0,   1,   0.20833333333333334,   -0.0034890329605839591,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;

// // v1 q=7.5
    m_K2c[1] <<
-0.040387419828745691,   -0.18438399006096629,   0.30068296075944245,   1.3691893625013596,   5.2174300453342751e-15,   -0.12174475829946513; // Q = 10, R-=200
//-0.42875437936449812,   -0.3169739912062795,   -1.0294664428425566,   0.04057357149091402,   -0.093311658910749912; // Q = 2.5      

    m_T[1] <<
-1.2468324983583301e-18,   1.463672932855431e-18,   5.4817261840867104e-16,   -3.9968028886505635e-15,   1,   0,
     0.00175403226095842,   -0.0032876961419837465,   -0.13920091722212505,   0.99025714795169484,   3.9690473130349346e-15,   0,
     0.32799131706503615,   -0.60922326435768381,   -0.7145966808848867,   -0.10305480951228874,   0,   0,
     -0.37795589684621772,   0.61579555995910384,   -0.68497389047703261,   -0.093573167834497095,   0,   0,
     0.8657759290601359,   0.49963189805505753,   -0.028026017868769525,   -0.0038143712952887769,   0,   0,
     0,   0,   0,   0,   0,   1;
   
    m_Gamma_aug[1] <<
    0,
     0,
     -0,
     -0,
     0,
     1;

/////////////////////////////////////////////////////////////// v2 ////////////////////////////////////////////////////////////////////////////
    m_phi_aug[2] <<   
0.87929873026351646,   -0.20962126769509487,   0,   0,   0,   0.79193149253770323,
     -0.0073194784724563006,   0.9117373321979696,   0,   0,   0,   0.45701690410326046,
     -0.013755613355515913,   -0.078674049008538458,   1,   0.20833333333333334,   0.021701388888888888,   -0.025635600870535839,
     5.6983253816493898e-05,   -0.014325591637869291,   0,   1,   0.20833333333333334,   -0.0034890329605839591,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;

// // v1 q=7.5
    m_K2c[2] <<
-0.040387419828745691,   -0.18438399006096629,   0.30068296075944245,   1.3691893625013596,   5.2174300453342751e-15,   -0.12174475829946513; // Q = 10, R-=200
//-0.42875437936449812,   -0.3169739912062795,   -1.0294664428425566,   0.04057357149091402,   -0.093311658910749912; // Q = 2.5      

    m_T[2] <<
-1.2468324983583301e-18,   1.463672932855431e-18,   5.4817261840867104e-16,   -3.9968028886505635e-15,   1,   0,
     0.00175403226095842,   -0.0032876961419837465,   -0.13920091722212505,   0.99025714795169484,   3.9690473130349346e-15,   0,
     0.32799131706503615,   -0.60922326435768381,   -0.7145966808848867,   -0.10305480951228874,   0,   0,
     -0.37795589684621772,   0.61579555995910384,   -0.68497389047703261,   -0.093573167834497095,   0,   0,
     0.8657759290601359,   0.49963189805505753,   -0.028026017868769525,   -0.0038143712952887769,   0,   0,
     0,   0,   0,   0,   0,   1;
   
    m_Gamma_aug[2] <<
    0,
     0,
     -0,
     -0,
     0,
     1;
  
/////////////////////////////////////////////////////////////// v3 ////////////////////////////////////////////////////////////////////////////
    m_phi_aug[3] <<   
0.77470057539929549,   -0.37543924990080774,   0,   0,   0,   1.392475385585584,
     -0.013109449902579749,   0.83279928127973402,   0,   0,   0,   0.86789975150845278,
     -0.025263182193746184,   -0.15050514571317819,   1,   0.41666666666666663,   0.086805555555555552,   -0.098846957334415717,
     0.00021194441614220494,   -0.027398713241840178,   0,   1,   0.41666666666666669,   -0.013479976627709949,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;


    // q=7.5
    m_K2c[3] <<
-0.033022535467495895,   -0.19943940068727697,   0.26984675232339539,   1.3816832755480373,   7.288652848725869e-15,   -0.23901788624189627; // Q = 7.5
//-0.29249027996323107,   -0.28549560532646351,   -1.056563344205941,   0.079929179840173781,   -0.18633734617443931; // Q = 2.5
    
    m_T[3] <<
2.7863995832877464e-17,   -5.1445393084437185e-17,   6.8001160258290838e-16,   -5.3290705182007514e-15,   1,   0,
     -0.0012436323235344717,   0.0022127958697312862,   -0.13324612449743253,   0.99107972798332311,   5.467848396278896e-15,   0,
     0.34869882213346731,   -0.63833967817361592,   -0.68036879129508521,   -0.089609678850010061,   0,   0,
     -0.40105616003502309,   0.56014480095206209,   -0.71814199542710977,   -0.098304795754358409,   0,   0,
     0.84709004321815673,   0.52797287882051147,   -0.060131959406364066,   -0.0082003273467879511,   0,   0,
     0,   0,   0,   0,   0,   1;

  

    m_Gamma_aug[3] <<
0,
     0,
     -0,
     -0,
     0,
     1;
   
/////////////////////////////////////////////////////////////// v4 ////////////////////////////////////////////////////////////////////////////
     m_phi_aug[4] <<   
0.87929873026351646,   -0.20962126769509487,   0,   0,   0,   0.79193149253770323,
     -0.0073194784724563006,   0.9117373321979696,   0,   0,   0,   0.45701690410326046,
     -0.013755613355515913,   -0.078674049008538458,   1,   0.20833333333333334,   0.021701388888888888,   -0.025635600870535839,
     5.6983253816493898e-05,   -0.014325591637869291,   0,   1,   0.20833333333333334,   -0.0034890329605839591,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;

// // v1 q=7.5
    m_K2c[4] <<
-0.040387419828745691,   -0.18438399006096629,   0.30068296075944245,   1.3691893625013596,   5.2174300453342751e-15,   -0.12174475829946513; // Q = 10, R-=200
//-0.42875437936449812,   -0.3169739912062795,   -1.0294664428425566,   0.04057357149091402,   -0.093311658910749912; // Q = 2.5      

    m_T[4] <<
-1.2468324983583301e-18,   1.463672932855431e-18,   5.4817261840867104e-16,   -3.9968028886505635e-15,   1,   0,
     0.00175403226095842,   -0.0032876961419837465,   -0.13920091722212505,   0.99025714795169484,   3.9690473130349346e-15,   0,
     0.32799131706503615,   -0.60922326435768381,   -0.7145966808848867,   -0.10305480951228874,   0,   0,
     -0.37795589684621772,   0.61579555995910384,   -0.68497389047703261,   -0.093573167834497095,   0,   0,
     0.8657759290601359,   0.49963189805505753,   -0.028026017868769525,   -0.0038143712952887769,   0,   0,
     0,   0,   0,   0,   0,   1;
   
    m_Gamma_aug[4] <<
    0,
     0,
     -0,
     -0,
     0,
     1;

/////////////////////////////////////////////////////////////// v5 ////////////////////////////////////////////////////////////////////////////
     m_phi_aug[5] <<   
0.87929873026351646,   -0.20962126769509487,   0,   0,   0,   0.79193149253770323,
     -0.0073194784724563006,   0.9117373321979696,   0,   0,   0,   0.45701690410326046,
     -0.013755613355515913,   -0.078674049008538458,   1,   0.20833333333333334,   0.021701388888888888,   -0.025635600870535839,
     5.6983253816493898e-05,   -0.014325591637869291,   0,   1,   0.20833333333333334,   -0.0034890329605839591,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;

// // v1 q=7.5
    m_K2c[5] <<
-0.040387419828745691,   -0.18438399006096629,   0.30068296075944245,   1.3691893625013596,   5.2174300453342751e-15,   -0.12174475829946513; // Q = 10, R-=200
//-0.42875437936449812,   -0.3169739912062795,   -1.0294664428425566,   0.04057357149091402,   -0.093311658910749912; // Q = 2.5      

    m_T[5] <<
-1.2468324983583301e-18,   1.463672932855431e-18,   5.4817261840867104e-16,   -3.9968028886505635e-15,   1,   0,
     0.00175403226095842,   -0.0032876961419837465,   -0.13920091722212505,   0.99025714795169484,   3.9690473130349346e-15,   0,
     0.32799131706503615,   -0.60922326435768381,   -0.7145966808848867,   -0.10305480951228874,   0,   0,
     -0.37795589684621772,   0.61579555995910384,   -0.68497389047703261,   -0.093573167834497095,   0,   0,
     0.8657759290601359,   0.49963189805505753,   -0.028026017868769525,   -0.0038143712952887769,   0,   0,
     0,   0,   0,   0,   0,   1;
   
    m_Gamma_aug[5] <<
    0,
     0,
     -0,
     -0,
     0,
     1;
    
/////////////////////////////////////////////////////////////// v6 ////////////////////////////////////////////////////////////////////////////
     m_phi_aug[6] <<   
0.87929873026351646,   -0.20962126769509487,   0,   0,   0,   0.79193149253770323,
     -0.0073194784724563006,   0.9117373321979696,   0,   0,   0,   0.45701690410326046,
     -0.013755613355515913,   -0.078674049008538458,   1,   0.20833333333333334,   0.021701388888888888,   -0.025635600870535839,
     5.6983253816493898e-05,   -0.014325591637869291,   0,   1,   0.20833333333333334,   -0.0034890329605839591,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;

// // v1 q=7.5
    m_K2c[6] <<
-0.040387419828745691,   -0.18438399006096629,   0.30068296075944245,   1.3691893625013596,   5.2174300453342751e-15,   -0.12174475829946513; // Q = 10, R-=200
//-0.42875437936449812,   -0.3169739912062795,   -1.0294664428425566,   0.04057357149091402,   -0.093311658910749912; // Q = 2.5      

    m_T[6] <<
-1.2468324983583301e-18,   1.463672932855431e-18,   5.4817261840867104e-16,   -3.9968028886505635e-15,   1,   0,
     0.00175403226095842,   -0.0032876961419837465,   -0.13920091722212505,   0.99025714795169484,   3.9690473130349346e-15,   0,
     0.32799131706503615,   -0.60922326435768381,   -0.7145966808848867,   -0.10305480951228874,   0,   0,
     -0.37795589684621772,   0.61579555995910384,   -0.68497389047703261,   -0.093573167834497095,   0,   0,
     0.8657759290601359,   0.49963189805505753,   -0.028026017868769525,   -0.0038143712952887769,   0,   0,
     0,   0,   0,   0,   0,   1;
   
    m_Gamma_aug[6] <<
    0,
     0,
     -0,
     -0,
     0,
     1;
/////////////////////////////////////////////////////////////// v7 ////////////////////////////////////////////////////////////////////////////
     m_phi_aug[7] <<   
0.87929873026351646,   -0.20962126769509487,   0,   0,   0,   0.79193149253770323,
     -0.0073194784724563006,   0.9117373321979696,   0,   0,   0,   0.45701690410326046,
     -0.013755613355515913,   -0.078674049008538458,   1,   0.20833333333333334,   0.021701388888888888,   -0.025635600870535839,
     5.6983253816493898e-05,   -0.014325591637869291,   0,   1,   0.20833333333333334,   -0.0034890329605839591,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;

// // v1 q=7.5
    m_K2c[7] <<
-0.040387419828745691,   -0.18438399006096629,   0.30068296075944245,   1.3691893625013596,   5.2174300453342751e-15,   -0.12174475829946513; // Q = 10, R-=200
//-0.42875437936449812,   -0.3169739912062795,   -1.0294664428425566,   0.04057357149091402,   -0.093311658910749912; // Q = 2.5      

    m_T[7] <<
-1.2468324983583301e-18,   1.463672932855431e-18,   5.4817261840867104e-16,   -3.9968028886505635e-15,   1,   0,
     0.00175403226095842,   -0.0032876961419837465,   -0.13920091722212505,   0.99025714795169484,   3.9690473130349346e-15,   0,
     0.32799131706503615,   -0.60922326435768381,   -0.7145966808848867,   -0.10305480951228874,   0,   0,
     -0.37795589684621772,   0.61579555995910384,   -0.68497389047703261,   -0.093573167834497095,   0,   0,
     0.8657759290601359,   0.49963189805505753,   -0.028026017868769525,   -0.0038143712952887769,   0,   0,
     0,   0,   0,   0,   0,   1;
   
    m_Gamma_aug[7] <<
    0,
     0,
     -0,
     -0,
     0,
     1;
	 
/////////////////////////////////////////////////////////////// v8 ////////////////////////////////////////////////////////////////////////////
     m_phi_aug[8] <<   
0.77470057539929549,   -0.37543924990080774,   0,   0,   0,   1.392475385585584,
     -0.013109449902579749,   0.83279928127973402,   0,   0,   0,   0.86789975150845278,
     -0.025263182193746184,   -0.15050514571317819,   1,   0.41666666666666663,   0.086805555555555552,   -0.098846957334415717,
     0.00021194441614220494,   -0.027398713241840178,   0,   1,   0.41666666666666669,   -0.013479976627709949,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;


    // q=7.5
    m_K2c[8] <<
-0.033022535467495895,   -0.19943940068727697,   0.26984675232339539,   1.3816832755480373,   7.288652848725869e-15,   -0.23901788624189627; // Q = 7.5
//-0.29249027996323107,   -0.28549560532646351,   -1.056563344205941,   0.079929179840173781,   -0.18633734617443931; // Q = 2.5
    
    m_T[8] <<
2.7863995832877464e-17,   -5.1445393084437185e-17,   6.8001160258290838e-16,   -5.3290705182007514e-15,   1,   0,
     -0.0012436323235344717,   0.0022127958697312862,   -0.13324612449743253,   0.99107972798332311,   5.467848396278896e-15,   0,
     0.34869882213346731,   -0.63833967817361592,   -0.68036879129508521,   -0.089609678850010061,   0,   0,
     -0.40105616003502309,   0.56014480095206209,   -0.71814199542710977,   -0.098304795754358409,   0,   0,
     0.84709004321815673,   0.52797287882051147,   -0.060131959406364066,   -0.0082003273467879511,   0,   0,
     0,   0,   0,   0,   0,   1;

  

    m_Gamma_aug[8] <<
0,
     0,
     -0,
     -0,
     0,
     1;


/////////////////////////////////////////////////////////////// origin ////////////////////////////////////////////////////////////////////////////
    m_phi_aug[9] <<   
0.80792438802720157,   -0.32456196047485225,   0,   0,   0,   1.2102236227283745,
     -0.011332933775056417,   0.85814989642400485,   0,   0,   0,   0.73506828763880749,
     -0.021656236021812294,   -0.12725181289665008,   1,   0.34722225000000001,   0.060281645447531257,   -0.069476382680105156,
     0.00015077767104498864,   -0.023171726926463871,   0,   1,   0.34722225000000001,   -0.009469023758363353,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;
    
    // q=7.5
    m_K2c[9] <<
-0.035357364156281559,   -0.19484710586020487,   0.27978720902320431,   1.3791572491215471,   2.1535830445830365e-14,   -0.20031870037760921; // Q = 7.5
//-0.28614679486699318,   -0.28802685785999194,   -1.0593314709112267,   0.080818517814108204,   -0.15246689570600142; // Q = 2.5
    

    m_T[9] <<
5.6297197806309818e-17,   -1.0100698489418081e-16,   2.0851376181241221e-15,   -1.6070478281449141e-14,   1,   -1.5104317985218285e-21,
     -0.00022207683140596998,   0.00035190023026594383,   -0.13518521998256777,   0.99082025773919857,   1.6209256159527285e-14,   4.022369879297404e-08,
     0.34167256248097966,   -0.62912948309980554,   -0.69181152594753803,   -0.094089138996978086,   0,   -5.5855723546412224e-06,
     -0.3931717175107009,   0.57907026545147589,   -0.70761278839476494,   -0.09683883530352437,   0,   9.9483087252228022e-05,
     0.85362425678491349,   0.51852982959429328,   -0.049050082207747661,   -0.0066851143215465505,   0,   -0.0013574723711328248,
     0.0011997948373507761,   0.00064276875255240076,   -4.7352358391183976e-08,   -6.4278426722387318e-09,   0,   0.99999907366990937;
  

    m_Gamma_aug[9] <<
0.0011997959487579943,
     0.00064276934796898908,
     -4.7352402255139051e-08,
     -6.4278486265483322e-09,
     0,
     1;

/////////////////////////////////////////////////////////////// 1 classifier ////////////////////////////////////////////////////////////////////////////
    m_phi_aug[10] <<   
0.7430117655074826,   -0.42226334307667951,   0,   0,   0,   1.2949789275057961,
     -0.014744434300681784,   0.80835642589938728,   0,   0,   0,   0.85264704541232605,
     -0.028656443477193112,   -0.17311895237573724,   1,   0.4861111500000001,   0.1181520250771613,   -0.13056175708352763,
     0.00028163630498058661,   -0.031501232649616101,   0,   1,   0.4861111500000001,   -0.017815731158167942,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;
    
    // q=7.5
    m_K2c[10] <<
-0.030863582279881424,   -0.19837794595954933,   0.25673786389239417,   1.3661981961267431,   5.8991791203775661e-15,   -0.23817541726666216; // Q = 7.5
//-0.28614679486699318,   -0.28802685785999194,   -1.0593314709112267,   0.080818517814108204,   -0.15246689570600142; // Q = 2.5
    

    m_T[10] <<
3.3827107781547738e-17,   -6.0390061007442597e-17,   5.7245874707234634e-16,   -4.6351811278100286e-15,   1.0000000000000002,   -3.2441361879839703e-19,
     -0.0029555991486657556,   0.0052472380319253066,   -0.13027553290234903,   0.99145953797206321,   4.4686476741162551e-15,   3.1905180228230615e-05,
     0.36319324884729798,   -0.65300660855322246,   -0.65948516331352769,   -0.082116058072350423,   0,   -0.0030505005411244103,
     -0.43266805803682923,   0.51279338358830695,   -0.73370795807016986,   -0.1004125138178146,   0,   0.037831952579580545,
     0.78633084087009886,   0.54021345056496528,   -0.098868536037532165,   -0.013496952948323033,   0,   -0.28265894885554771,
     0.25012814451112764,   0.13699346428061454,   -0.0022912316853693415,   -0.00031128889980046282,   0,   0.95846927724596442;
  

    m_Gamma_aug[10] <<
0.26096626198581763,
     0.14292942667317127,
     -0.0023905113494643196,
     -0.00032477712868889298,
     0,
     1;

/////////////////////////////////////////////////////////////// 2 classifier ////////////////////////////////////////////////////////////////////////////
    m_phi_aug[11] <<   
0.71278252000766262,   -0.46527340465977157,   0,   0,   0,   1.4698251193790384,
     -0.016246243628149314,   0.78478292713596276,   0,   0,   0,   0.99157825520539877,
     -0.031846711792395996,   -0.19512883479153587,   1,   0.55555560000000004,   0.15432101234568002,   -0.16973665757915213,
     0.00035916681001676271,   -0.035483726547048483,   0,   1,   0.55555560000000004,   -0.023171322809509946,
     0,   0,   0,   0,   1,   0,
     0,   0,   0,   0,   0,   0;
    
    // q=7.5
    m_K2c[11] <<
-0.028745177801733357,   -0.20277739860807659,   0.24792362739226767,   1.3674431356744219,   -2.1717832243375372e-14,   -0.2795460924718513; // Q = 7.5
//-0.28614679486699318,   -0.28802685785999194,   -1.0593314709112267,   0.080818517814108204,   -0.15246689570600142; // Q = 2.5
    

    m_T[11] <<
-1.5558301175166989e-16,   2.7798943702528334e-16,   -1.9394208461420703e-15,   1.6167622796103842e-14,   0.99999999999999989,   1.5077186461126546e-18,
     -0.0039798640078039689,   0.0070569631791351688,   -0.12855237555702123,   0.99166962494540511,   -1.6417422976644502e-14,   4.0650380086368302e-05,
     0.36999123171434656,   -0.66007707353115286,   -0.64908793176565116,   -0.077960441428738397,   0,   -0.0034310146421961522,
     -0.43928171968414587,   0.49500716640278059,   -0.74180151422911134,   -0.10144852153327334,   0,   0.037923435067778899,
     0.78700210779474122,   0.55142349620984954,   -0.1090243486904888,   -0.01488825767276732,   0,   -0.25387367199145427,
     0.22527738948894593,   0.12307919932642228,   -0.0018298783697068865,   -0.00024858611710907105,   0,   0.96648755720304391;
  

    m_Gamma_aug[11] <<
0.23308876333688644,
     0.12734690520238665,
     -0.0018933284304274366,
     -0.00025720570870923999,
     0,
     1;
}
// destructor
lateralController50_bev::~lateralController50_bev(){}

// class methods
void lateralController50_bev::compute_steering_angles(long double the_yL, int the_it_counter, int the_pipe_version) {
    cout<<"changeto50"<<endl;
    m_z3 = the_yL;
    m_z5 = 2 * m_z3 / ( pow( m_LL + m_l_r, 2 ) );   // curvature calculate

    Matrix<long double, 6, 1> zt_temp, zt;       // zt is the transferred state vector
    if (the_it_counter == 0){
        zt_temp <<  m_z1,
                    m_z2,
                    m_z3,
                    m_z4,
                    m_z5,
                    0.0L;
    } else {
        zt_temp <<  m_z1,
                    m_z2,
                    m_z3,
                    m_z4,
                    m_z5,
                    m_input[the_it_counter-1];
    }

    // zt = m_T[the_pipe_version] * zt_temp;        

    // Matrix<long double, 6, 1> zt_temp_2;
    // zt_temp_2 << zt[1], 
    //              zt[2], 
    //              zt[3], 
    //              zt[4], 
    //              zt[5],
    //              zt[6];

    // calculate the desired steering angle 
    m_desired_steering_angle = m_K2c[the_pipe_version] * zt_temp;   
    // calculate left front tire steering angle according to desired steering angle                     
    m_steering_angle_left    = atan( m_l / (-m_d + m_l / tan(m_desired_steering_angle) ) );  
    // calculate right front tire steering angle according to desired steering angle 
    m_steering_angle_right   = atan( m_l / ( m_d + m_l / tan(m_desired_steering_angle) ) );         
}

long double lateralController50_bev::get_steering_angles() {
    // return steering angles
    long double steering_angles = 0.0L;
    steering_angles =  m_desired_steering_angle;
    // steering_angles[0] = m_steering_angle_left;
    // steering_angles[1] = m_steering_angle_right;
    return steering_angles;       
}

void lateralController50_bev::estimate_next_state(int the_it_counter, int the_pipe_version) {
    // transfer state vector
    Matrix<long double, 6, 1> zkp_temp, zkp;  
    if (the_it_counter == 0){
        zkp_temp << m_z1,
                    m_z2,
                    m_z3,
                    m_z4,
                    m_z5,
                    0.0L;       
    } else {
        zkp_temp << m_z1,
                    m_z2,
                    m_z3,
                    m_z4,
                    m_z5,
                    m_input[the_it_counter-1]; 
    }

    // given the control design, estimate next states
    zkp = m_phi_aug[the_pipe_version]   * zkp_temp + 
          m_Gamma_aug[the_pipe_version] * m_desired_steering_angle;  

        m_z1 = zkp[0];
        m_z2 = zkp[1];
        m_z4 = zkp[3];
        m_z5 = zkp[4];
        m_input[the_it_counter+1] = m_desired_steering_angle;
}

vector<long double> lateralController50_bev::get_steering_angle_left_container(){
    return m_steering_angle_left_container;
}

vector<long double> lateralController50_bev::get_steering_angle_right_container(){
    return m_steering_angle_right_container;
}

