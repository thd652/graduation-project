# verilog-A Modeling : A 3-Bit Self-Rectifying Resistive Memory Physics-based Compact Model 

# Code - 0에서 1.7V(positive)
```
// VerilogA for MJH, RR1, veriloga

`include "constants.vams"
`include "disciplines.vams"
`include "constants.h"

module RR1(p,n);
	inout p;
	inout n;
	electrical p, n, mid;

////////// Parameter
	parameter real G_H_F=0.5415;
	parameter real G_H_R=0.3985;//
	parameter real G_L1_F=0.7015;
	parameter real G_L1_R=0.3985;//
	parameter real G_L2_F=0.6195;
	parameter real G_L2_R=0.3985;//
	parameter real G_L3_F=0.6187;
	parameter real G_L3_R=0.3985;//
	parameter real G_L4_F=0.6184; 
	parameter real G_L4_R=0.3985;// 
	parameter real G_L5_F=0.6181; 
	parameter real G_L5_R=0.3985;//
	parameter real G_L6_F=0.6179; 
	parameter real G_L6_R=0.3985;//
	parameter real G_L7_F=0.6177;  
	parameter real G_L7_R=0.3985;//
 
	parameter real V_EMF=0.2;
	parameter real Vth_p=1.69999;
	parameter real Vth_n=0.99999;
	parameter real Vt=26e-3;
	parameter real current_state_initial=0;
	parameter real alpha=1e11;
	parameter real beta=0;
	integer seed;

	real drdt, r_normal, r_normal_last, first_iteration, stop;
	real R,  R_LRS7,  R_LRS6, R_LRS5, R_LRS4, R_LRS3, R_LRS2, R_LRS1, R_HRS;
	real Is_L3_F, Is_L3_R, Is_L2_F, Is_L2_R, Is_L1_F, Is_L1_R, Is_H_F, Is_H_R;
	real Is_L7_F, Is_L7_R, Is_L6_F, Is_L6_R, Is_L5_F, Is_L5_R, Is_L4_F, Is_L4_R;
	real mismatch_L, mismatch_H, time_retention_L, time_retention_H;
	real cycling_endurance_L, cycling_endurance_H, endurance;
	real current_state, R_state, stop_state;
	real w1, w2, w3, w4, w5, w6, w7;

	analog begin
		if (first_iteration == 0) 
			begin
				if (V(p,n) >= Vth_p) //set start 
					begin
						r_normal_last = 1;
					end
				else if (V(p,n) <= -Vth_n) //reset start 
					begin
						r_normal_last = 0;
					end
				else 
					begin
						r_normal_last = 0;
					end
////////// mismatch
				seed = $random;
				mismatch_L = $rdist_normal(seed, 1, 0.11);  ///+-10%
				mismatch_H = $rdist_normal(seed, 1, 0.03);  ///+-3%
				cycling_endurance_L = 1;
				cycling_endurance_H = 1;
				endurance = 0;
				current_state = current_state_initial;
				R_state = 0;
				first_iteration = 1;
				stop_state = 1;
			end
////////// Is
		Is_H_F = 2.5488e-18;
		Is_H_R = 4.8145e-16;//
		Is_L1_F = 1.4960e-18;
		Is_L1_R = 5.0455e-16;//
		Is_L2_F = 5.0757e-17;
		Is_L2_R = 5.2865e-16;//
		Is_L3_F = 7.8353e-17;
		Is_L3_R = 5.5345e-16;//
		Is_L4_F = 1.0636e-16;
		Is_L4_R = 5.7945e-16;//
		Is_L5_F = 1.3386e-16;
		Is_L5_R = 6.0645e-16;//
		Is_L6_F = 1.6417e-16;
		Is_L6_R = 6.3545e-16;//
		Is_L7_F = 1.9350e-16;
		Is_L7_R = 6.6445e-16;//

////////// dr/dt, current_state Update
		if (V(p,n) >= Vth_p)
			begin
				drdt = alpha * (V(p,n) - Vth_p);
				if (stop_state==1)
					begin
						current_state = current_state + 1;	
						stop_state=0;
					end
			end

		else if (V(p,n) <= -Vth_n)
			begin
				drdt = alpha * (V(p,n) + Vth_n);
				if (stop_state==1)
					begin
						current_state = current_state - 1;	
						stop_state=0;
					end
				
			end

		else 
			begin
				drdt = beta * V(p,n);
				stop_state=1;
			end
				
////////// current_state Block
		if (current_state == 8)
			begin
				current_state = 7;
			end
		else if (current_state == -1)
			begin
				current_state = 0;
			end

////////// r_normal Update 
		r_normal = idt(drdt, r_normal_last, stop);

		if ((r_normal >= 1) && (V(p,n) > 0))
			begin
				r_normal_last = 1;
				stop = 1;
			end
		else if ((r_normal <= 0) && (V(p,n) < 0))
			begin
				r_normal_last = 0;
				stop = 1;
			end
		else if (stop != 0)
			begin
				r_normal_last = r_normal;
				stop = 0;
			end

		if (r_normal >= 0.5)
			begin
				r_normal = 1;
			end
		else
			begin
				r_normal = 0;
			end

////////// R_state Update
		if (r_normal_last == 0)
			begin
				R_state = current_state + 1;
			end
		else // r_normal_last == 1
			begin
				R_state = current_state;
			end

////////// cycling_endurance
		if ((V(p,n) < 0.8) && (endurance == 0))
			begin
				endurance = 1;
				cycling_endurance_L = cycling_endurance_L + 0.00005;
				cycling_endurance_H = cycling_endurance_H - 0.00004;
			end
		else if ((V(p,n) > 0.8) && (endurance == 1))
			begin
				endurance = 0;
			end

////////// time_retention
		time_retention_L = 1 - 0.15 * log($abstime/1728 + 1);
		time_retention_H = 1 - 0.225 * log($abstime/1728 + 1);
		// time_retention_L = 1;
		// time_retention_H = 1;



/*
////////// Positive(20T-1)
		Is_H_P_F = 2.5417e-18;
		Is_H_P_R = 5.9297e-15;
		Is_L1_P_F = 1.4960e-18;
		Is_L1_P_R = 6.2091e-15;
		Is_L2_P_F = 3.2436e-18;
		Is_L2_P_R = 6.5021e-15;
		Is_L3_P_F = 5.0978e-18;
		Is_L3_P_R = 6.8091e-15;
		Is_L4_P_F = 7.0205e-18;
		Is_L4_P_R = 7.1291e-15;
		Is_L5_P_F = 8.9316e-18;
		Is_L5_P_R = 7.4661e-15;
		Is_L6_P_F = 1.1073e-17;
		Is_L6_P_R = 7.8175e-15;
		Is_L7_P_F = 1.3151e-17;
		Is_L7_P_R = 8.1865e-15;
*/

////////// Is  total variation
		// Is_L_F = Is_L_F * mismatch_L * time_retention_L * cycling_endurance_L;
		// Is_L_R = Is_L_R * mismatch_L * time_retention_L * cycling_endurance_L;
		// Is_H_F = Is_H_F * mismatch_H * time_retention_H * cycling_endurance_H;
		// Is_H_R = Is_H_R * mismatch_H * time_retention_H * cycling_endurance_H;

////////// R Parameter
		R_HRS = 12000;
		R_LRS1 = 11500;
		R_LRS2 = 11000;
		R_LRS3 = 10500;
		R_LRS4 = 10000;
		R_LRS5 = 9500;
		R_LRS6 = 9000;
		R_LRS7 = 8500;


		// R_LRS = R_LRS * (2 - mismatch_L) * (2 - time_retention_L) * (2 - cycling_endurance_L);
		// R_HRS = R_HRS * (2 - mismatch_H) * (2 - time_retention_H) * (2 - cycling_endurance_H);

////////// R
		if (R_state == 1)
			begin
				w1 = r_normal;
				R = R_HRS * pow(R_LRS1 / R_HRS, w1);
			end
		else if (R_state == 2)
			begin
				w2 = r_normal;
				R = R_LRS1 * pow(R_LRS2 / R_LRS1, w2);
			end
		else if (R_state == 3)
			begin
				w3 = r_normal;
				R = R_LRS2 * pow(R_LRS3 / R_LRS2, w3);
			end
		else if (R_state == 4)
			begin
				w4 = r_normal;
				R = R_LRS3 * pow(R_LRS4 / R_LRS3, w4);
			end
		else if (R_state == 5)
			begin
				w5 = r_normal;
				R = R_LRS4 * pow(R_LRS5 / R_LRS4, w5);
			end
		else if (R_state == 6)
			begin
				w6 = r_normal;
				R = R_LRS5 * pow(R_LRS6 / R_LRS5, w6);
			end
		else if (R_state == 7)
			begin
				w7 = r_normal;
				R = R_LRS6 * pow(R_LRS7 / R_LRS6, w7);
			end
		else if (R_state == 8)
			begin
				w7 = r_normal;
				R = R_LRS6 * pow(R_LRS7 / R_LRS6, w7);
			end


////////// I
		if (current_state == 0)  // HRS
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_H_F * (exp(G_H_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_H_R * (exp(G_H_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 1)  // LRS1
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L1_F * (exp(G_L1_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L1_R * (exp(G_L1_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 2)  // LRS2
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L2_F * (exp(G_L2_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L2_R * (exp(G_L2_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 3)  // LRS3
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L3_F * (exp(G_L3_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L3_R * (exp(G_L3_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 4)  // LRS4
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L4_F * (exp(G_L4_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L4_R * (exp(G_L4_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 5)  // LRS5
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L5_F * (exp(G_L5_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L5_R * (exp(G_L5_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 6)  // LRS6
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L6_F * (exp(G_L6_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L6_R * (exp(G_L6_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end	

		if (current_state == 7)  // LRS7
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L7_F * (exp(G_L7_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L7_R * (exp(G_L7_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end	


		first_iteration=1;
		end
endmodule
```
# Code - 0에서 -1V(negative)
```
// VerilogA for MJH, RR1, veriloga

`include "constants.vams"
`include "disciplines.vams"
`include "constants.h"

module RR1(p,n);
	inout p;
	inout n;
	electrical p, n, mid;

////////// Parameter
////////// Is fix
	parameter real G_H_F=0.1386;
	parameter real G_H_R=0.0825;
	parameter real G_L1_F=0.1348;
	parameter real G_L1_R=0.0796;
	parameter real G_L2_F=0.1311;
	parameter real G_L2_R=0.0771;
	parameter real G_L3_F=0.1273;
	parameter real G_L3_R=0.0763;
	parameter real G_L4_F=0.1236;
	parameter real G_L4_R=0.0757;
	parameter real G_L5_F=0.1198;
	parameter real G_L5_R=0.0754; 
	parameter real G_L6_F=0.1160;
	parameter real G_L6_R=0.0748;
	parameter real G_L7_F=0.1122;
	parameter real G_L7_R=0.0743;

	parameter real V_EMF=-0.1;
	parameter real Vth_p=1.69999;
	parameter real Vth_n=0.99999;
	parameter real Vt=26e-3;
	parameter real current_state_initial=0;
	parameter real alpha=1e11;
	parameter real beta=0;
	integer seed;
	real drdt, r_normal, r_normal_last, first_iteration, stop;
	real R,  R_LRS7,  R_LRS6, R_LRS5, R_LRS4, R_LRS3, R_LRS2, R_LRS1, R_HRS;
	real Is_L3_F, Is_L3_R, Is_L2_F, Is_L2_R, Is_L1_F, Is_L1_R, Is_H_F, Is_H_R;
	real Is_L7_F, Is_L7_R, Is_L6_F, Is_L6_R, Is_L5_F, Is_L5_R, Is_L4_F, Is_L4_R;
	real mismatch_L, mismatch_H, time_retention_L, time_retention_H;
	real cycling_endurance_L, cycling_endurance_H, endurance;
	real current_state, R_state, stop_state;
	real w1, w2, w3, w4, w5, w6, w7;

	analog begin
		if (first_iteration == 0) 
			begin
				if (V(p,n) >= Vth_p) //set start 
					begin
						r_normal_last = 1;
					end
				else if (V(p,n) <= -Vth_n) //reset start 
					begin
						r_normal_last = 0;
					end
				else 
					begin
						r_normal_last = 0;
					end
////////// mismatch
				endurance = 0;
				current_state = current_state_initial;
				R_state = 0;
				first_iteration = 1;
				stop_state = 1;
			end
////////// Is
////////// Is fix
		Is_H_F = 1.1672e-13;//
		Is_H_R = 1.1874e-13;
		Is_L1_F = 1.1672e-13;//
		Is_L1_R = 1.1874e-13;
		Is_L2_F = 1.1672e-13;//
		Is_L2_R = 1.1874e-13;
		Is_L3_F = 1.1672e-13;//
		Is_L3_R = 1.1874e-13;
		Is_L4_F = 1.1672e-13;//
		Is_L4_R = 1.1874e-13;
		Is_L5_F = 1.1672e-13;//
		Is_L5_R = 1.1874e-13;
		Is_L6_F = 1.1672e-13;//
		Is_L6_R = 1.1874e-13;
		Is_L7_F = 1.1672e-13;//
		Is_L7_R = 1.1874e-13;
////////// dr/dt, current_state Update
		if (V(p,n) >= Vth_p)
			begin
				drdt = alpha * (V(p,n) - Vth_p);
				if (stop_state==1)
					begin
						current_state = current_state + 1;	
						stop_state=0;
					end
			end

		else if (V(p,n) <= -Vth_n)
			begin
				drdt = alpha * (V(p,n) + Vth_n);
				if (stop_state==1)
					begin
						current_state = current_state - 1;	
						stop_state=0;
					end
				
			end

		else 
			begin
				drdt = beta * V(p,n);
				stop_state=1;
			end
				
////////// current_state Block
		if (current_state == 8)
			begin
				current_state = 7;
			end
		else if (current_state == -1)
			begin
				current_state = 0;
			end

////////// r_normal Update 
		r_normal = idt(drdt, r_normal_last, stop);

		if ((r_normal >= 1) && (V(p,n) > 0))
			begin
				r_normal_last = 1;
				stop = 1;
			end
		else if ((r_normal <= 0) && (V(p,n) < 0))
			begin
				r_normal_last = 0;
				stop = 1;
			end
		else if (stop != 0)
			begin
				r_normal_last = r_normal;
				stop = 0;
			end

		if (r_normal >= 0.5)
			begin
				r_normal = 1;
			end
		else
			begin
				r_normal = 0;
			end

////////// R_state Update
		if (r_normal_last == 0)
			begin
				R_state = current_state + 1;
			end
		else // r_normal_last == 1
			begin
				R_state = current_state;
			end

////////// cycling_endurance
		if ((V(p,n) < 0.8) && (endurance == 0))
			begin
				endurance = 1;
				cycling_endurance_L = cycling_endurance_L + 0.00005;
				cycling_endurance_H = cycling_endurance_H - 0.00004;
			end
		else if ((V(p,n) > 0.8) && (endurance == 1))
			begin
				endurance = 0;
			end

////////// time_retention
		time_retention_L = 1 - 0.15 * log($abstime/1728 + 1);
		time_retention_H = 1 - 0.225 * log($abstime/1728 + 1);
		// time_retention_L = 1;
		// time_retention_H = 1;


////////// Is  total variation
		// Is_L_F = Is_L_F * mismatch_L * time_retention_L * cycling_endurance_L;
		// Is_L_R = Is_L_R * mismatch_L * time_retention_L * cycling_endurance_L;
		// Is_H_F = Is_H_F * mismatch_H * time_retention_H * cycling_endurance_H;
		// Is_H_R = Is_H_R * mismatch_H * time_retention_H * cycling_endurance_H;

////////// R Parameter
		R_HRS = 12000;
		R_LRS1 = 11500;
		R_LRS2 = 11000;
		R_LRS3 = 10500;
		R_LRS4 = 10000;
		R_LRS5 = 9500;
		R_LRS6 = 9000;
		R_LRS7 = 8500;


		// R_LRS = R_LRS * (2 - mismatch_L) * (2 - time_retention_L) * (2 - cycling_endurance_L);
		// R_HRS = R_HRS * (2 - mismatch_H) * (2 - time_retention_H) * (2 - cycling_endurance_H);

////////// R
		if (R_state == 1)
			begin
				w1 = r_normal;
				R = R_HRS * pow(R_LRS1 / R_HRS, w1);
			end
		else if (R_state == 2)
			begin
				w2 = r_normal;
				R = R_LRS1 * pow(R_LRS2 / R_LRS1, w2);
			end
		else if (R_state == 3)
			begin
				w3 = r_normal;
				R = R_LRS2 * pow(R_LRS3 / R_LRS2, w3);
			end
		else if (R_state == 4)
			begin
				w4 = r_normal;
				R = R_LRS3 * pow(R_LRS4 / R_LRS3, w4);
			end
		else if (R_state == 5)
			begin
				w5 = r_normal;
				R = R_LRS4 * pow(R_LRS5 / R_LRS4, w5);
			end
		else if (R_state == 6)
			begin
				w6 = r_normal;
				R = R_LRS5 * pow(R_LRS6 / R_LRS5, w6);
			end
		else if (R_state == 7)
			begin
				w7 = r_normal;
				R = R_LRS6 * pow(R_LRS7 / R_LRS6, w7);
			end
		else if (R_state == 8)
			begin
				w7 = r_normal;
				R = R_LRS6 * pow(R_LRS7 / R_LRS6, w7);
			end

////////// I
		if (current_state == 0)  // HRS
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_H_F * (exp(G_H_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_H_R * (exp(G_H_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 1)  // LRS1
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L1_F * (exp(G_L1_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L1_R * (exp(G_L1_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 2)  // LRS2
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L2_F * (exp(G_L2_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L2_R * (exp(G_L2_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 3)  // LRS3
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L3_F * (exp(G_L3_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L3_R * (exp(G_L3_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 4)  // LRS4
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L4_F * (exp(G_L4_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L4_R * (exp(G_L4_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 5)  // LRS5
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L5_F * (exp(G_L5_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L5_R * (exp(G_L5_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end

		if (current_state == 6)  // LRS6
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L6_F * (exp(G_L6_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L6_R * (exp(G_L6_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end	

		if (current_state == 7)  // LRS7
			begin
				if (V(p,n) >=  V_EMF)  // forward (Va >= V_EMF)
					begin
						I(p,n) <+ Is_L7_F * (exp(G_L7_F * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				else // reverse (Va < V_EMF)
					begin
						I(p,n) <+ Is_L7_R * (exp(G_L7_R * sqrt(abs(V(p,mid) - V_EMF)) / Vt));
					end
				V(mid,n) <+ I(p,n) * R;
			end	


		first_iteration=1;
		end
endmodule
```


