function Umbelical(length, diameter, specificWeight, eModulus, gModulus, handles){
	//to do:
	//update CoCalc sheet
	//update documentation
	//check signs of the Force and Hessian
	//add extra solvers
	//add option for two handle wire generation
	//show axis on handles
	//optimize inversion for banded matrix
	
	/**
	* Computes the shape of an umbelical
	*
	* Procedure according to: M.Grégoire and E.Schömer; Interactive simulation of one-dimensional flexible parts
	*
	* handles, 8 element: v, x_x, x_y, x_z, q_0, q_1, q_2, q_3
	* handles only act on the cylinderical discretization intervals
	* at least one handle has to be defined in order to run the simulation
	*
	* if only one handle is defined, the umbelical is hung in the direction of the quaternion with the handle at the given location.
	* if two or more handles are defined, the umbelical is spun between the two handles, with any exces length on both sides going in the direction between the two defined handles
	*
	* 
	* quaternions are defined in relation to ref = (0,1,0)
	*/
  
	this.length = length;
	this.diameter = diameter;
	this.specificWeight = specificWeight;
	this.eModulus = eModulus;
	this.gModulus = gModulus;
	//handles = handles;
	
	//test handle: : [0.0, 5.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0]
	var handles = [math.zeros(8,1),math.zeros(8,1)];
	/*
	//Test block
	handles[0]._data[0][0] = 0.01;
	handles[0]._data[1][0] = 0.0;
	handles[0]._data[2][0] = -.15;
	handles[0]._data[3][0] = 0.1;
	handles[0]._data[4][0] = 1.0;
	//handles[0]._data[4][0] = 0.9238795325112866;
	handles[0]._data[5][0] = 0.0;
	handles[0]._data[6][0] = 0.0;
	handles[0]._data[7][0] = 0.0;
	//handles[0]._data[7][0] = 0.3826834323650898;

	handles[1]._data[0][0] = 0.99;
	handles[1]._data[1][0] = 0.0;
	handles[1]._data[2][0] = 0.15;
	handles[1]._data[3][0] = 0.1;
	handles[1]._data[4][0] = 0.9238795325112866;
	handles[1]._data[5][0] = 0.0;
	handles[1]._data[6][0] = 0.0;
	handles[1]._data[7][0] = 0.3826834323650898;
	*/
	
	handles[0]._data[0][0] = 0.01;
	handles[0]._data[1][0] = -.1;
	handles[0]._data[2][0] = 0.0;
	handles[0]._data[3][0] = 0.1;
	handles[0] = math.subset(handles[0], math.index(math.range(4,8),0),quaternionDirection([ 1.0, 0.0, 0.0]));
				 
	handles[1]._data[0][0] = 0.99;
	handles[1]._data[1][0] = 0.1;
	handles[1]._data[2][0] = 0.0;
	handles[1]._data[3][0] = 0.2;
    handles[1] = math.subset(handles[1], math.index(math.range(4,8),0),quaternionDirection([ 0.0, 0.0, 1.0]));
	
	//model settings
	var n = 50; //number of discretization points
	var epsilon = 1e-6;
	var gamma = 1.0;
	//var k_Length = this.eModulus * (math.PI * math.pow(this.diameter/2,2)); //needs to be tuned
	var k_Length = 124 * 1e12 * (math.PI * math.pow(this.diameter/2,2)) * ((n-1)/length);
	var k_I_Length = 1 * k_Length;
	var k_Coh = 1e7 * (length/(n-1));
	k_Coh = 1e7 * (length/(n-1));
	var k_I_Coh = 1 * k_Coh;
	var k_QuatNorm = 1e7 * (length/(n-1));
	k_QuatNorm = 1e7 * (length/(n-1));
	var k_I_QuatNorm = 1 * k_QuatNorm;
	var k_Handles = 1e12;
	var g = math.matrix([[0],[0],[9.81]]) //gravity constant
	
	//model internal variables
	var m = (this.length * math.PI *  math.pow(this.diameter/2,2) * this.specificWeight) / n; //mass of each discretization point
	var L_ref = this.length / (n - 1); //reference length of the discretization interval
	var B_kappa = (length/(n-2)) * this.eModulus * (math.PI * math.pow(this.diameter/2,4)) / 4; //bending stiffness
	var B_tau = (length/(n-2)) * this.gModulus * (math.PI * math.pow(this.diameter/2,4)) / 2; //torsional stiffness
	var A = math.matrix([[B_kappa,0,0],[0,B_tau,0],[0,0,B_kappa]]); //bending and torsional stiffness matrix
	var H = math.zeros((7 * n) - 4, (7 * n) - 4); //Hessian matrix
	var F = math.zeros((7 * n) - 4,1); //Force vector
	var X = math.zeros((7 * n) - 4,1); //Position/Quaternion vector
	var f_iI = new Array(n-1); 
	var f_cI = new Array(n-1);
	var f_qI = new Array(n-1);

	//temporal variables	
    var L = new Array(n-1); //Length of a section
	var u = new Array(n-1); //Unit vector of a section
	var r = new Array(n-1); //Unit vector representation of the quaternion of a section
	var omega = new Array(n-1); //Darboux vector between two sections, representing bending & torsion between two sections
	
	var H_Length = math.zeros(10,10);
	var H_QuatNorm = math.zeros(7,7);
	var H_Coherence = math.zeros(10,10);
	var H_BendingTorsion = math.zeros(14, 14);
	var H_Handles = math.zeros(10,10);
	var H_Buffer = math.zeros(14,14);

	//generate positions
	if(handles.length == 0){
		throw new Error('No handles defined')
	}
	//if(handles.length == 1){
	if(handles.length > 0){
		//calculate direction of the initial umbelical
		let v = math.zeros(3,1);
		v._data[0][0] = 2 * (handles[0]._data[5][0] * handles[0]._data[6][0] - handles[0]._data[4][0] * handles[0]._data[7][0]);
		v._data[1][0] = math.pow(handles[0]._data[4][0],2) - math.pow(handles[0]._data[5][0],2) + math.pow(handles[0]._data[6][0],2) - math.pow(handles[0]._data[7][0],2);
		v._data[2][0] = 2 * (handles[0]._data[4][0] * handles[0]._data[5][0] + handles[0]._data[6][0] * handles[0]._data[7][0]);
		//calculate the start of the umbelical
		X = math.subset(X, math.index(math.range(0,3),0), math.subtract(math.subset(handles[0],math.index(math.range(1,4),0)), math.multiply(handles[0]._data[0][0], math.multiply(v, this.length))));
		
		//calculate the respective positions
		for(let i = 1; i < n; i++){
			X = math.subset(X, math.index(math.range(i*7,i*7+3),0), math.add(math.subset(X, math.index(math.range(0,3),0)), math.multiply(i * L_ref, v))); //calculate the subsequent coordinates based on the start, the direction and the interval
		}
		
		//generate L_i, u_i, r_i, omega_i for fast computation
		updateReusableVar();
		eliminateCoherence();
	}
/*
	if(handles.length > 1){
		throw new Error('Not yet implemented')
	}
*/	for(let i = 0; i < n-1; i++){	
		f_iI[i] = 0;
		f_cI[i] = math.zeros(3,1);
		f_qI[i] = 0;
	}
	/**
	* mnemonics
	*
	**/
	
	this.update = function(){
		solver:{
			updateReusableVar();
			F = math.add(math.add(math.add(getFweight(),math.add(getFlength(),getFIlength())),math.add(math.add(getFcoherence(),getFIcoherence()), math.add(getFquatnorm(),getFIquatnorm()))),math.add(getFbendingtorsion(),getFhandles()));
			H = math.add(math.add(math.add(getHlength(),getHIlength()),math.add(math.add(getHcoherence(),getHIcoherence()), math.add(getHquatnorm(),getHIquatnorm()))),math.add(getHbendingtorsion(),getHhandles()));		

			//Newton
			X_new = math.subtract(X,math.multiply(gamma, math.multiply(math.inv(H),F)));
			if (getEnergy(X_new) < getEnergy(X)){
				console.log("Newton:", getEnergy(X), getEnergy(X_new));
				X = X_new
				break solver;
			};
			
			//Steepest descent
			let alpha = math.divide(math.multiply(math.transpose(F),F),math.multiply(math.multiply(math.transpose(F), H),F));
			X_new = math.add(X, math.multiply(math.squeeze(alpha), F))
			if (getEnergy(X_new) < getEnergy(X)){
				console.log("Steepest descent:", getEnergy(X), getEnergy(X_new));
				X = X_new
				break solver;
			};
			
			//Line search
		    for(let k = 0; k < 50; k++){
				alpha = 1 / math.pow(2,k);
				X_new = math.add(X, math.multiply(gamma,math.multiply(math.squeeze(alpha), F)))
				if (getEnergy(X_new) < getEnergy(X)){
					console.log("Line search:", k, getEnergy(X), getEnergy(X_new));
					X = X_new
					break solver;
				};
			};
			
			//Update integral components
			console.log("Updating integral components");
			for(let i = 0; i < n-1; i++){	
				f_iI[i] = f_iI[i] + k_I_Length * (L[i] - L_ref);
				if (i < n - 1){
					f_cI[i] = math.add(f_cI[i], math.multiply(k_I_Coh, math.subtract(u[i], r[i])));
					f_qI[i] = f_qI[i] + k_I_QuatNorm * ((1/math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))))-1);
				}
			}
		}
		
	
	}

	this.display = function(){

		noStroke();
		E = filterPositions(getEbending());

		//remove the points around the handles as they suffer coherence/handle interaction
		E._data[0][0] = E._data[2][0];
		E._data[1][0] = E._data[2][0];
		E._data[n-2][0] = E._data[n-3][0];
		E._data[n-1][0] = E._data[n-3][0];

		for(let i = 0; i < n; i++){
			//ambientMaterial(127);
			let c = mapRGB(E._data[i][0],math.min(E),math.max(E));
			specularMaterial(c);
			mPush();
			mTranslate(X._data[i*7][0],X._data[i*7+1][0],X._data[i*7+2][0]);
			sphere(diameter/2);
			mPop();
		}
		
		for(let i = 0; i < n-1; i++){
			let c = mapRGB((E._data[i][0]+E._data[i+1][0])/2,math.min(E),math.max(E));
			specularMaterial(c);
			L[i] = math.norm(math.squeeze(math.subtract(math.subset(X, math.index(math.range(i*7+7,i*7+10),0)), math.subset(X, math.index(math.range(i*7,i*7+3),0))))); //L[i] = L[i] = |x[i+1]-x[i]|
			mPush();
			mTranslate((X._data[i*7][0] + X._data[i*7+7][0])/2,(X._data[i*7+1][0] + X._data[i*7+8][0])/2,(X._data[i*7+2][0] + X._data[i*7+9][0])/2);
			let q = math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)));
			q = math.divide(q, math.norm(q));
			if (q._data[0] != 1){
				mRotate(2*math.acos(q._data[0]), createVector(q._data[1] / math.sqrt(1 - math.pow(q._data[0],2)), q._data[2] / math.sqrt(1 - math.pow(q._data[0],2)),q._data[3] / math.sqrt(1 - math.pow(q._data[0],2))));
			}
			cylinder(diameter/2,L[i]);
			mPop();
		}
		
		ambientMaterial(32);
		specularMaterial(32);
		for(let j = 0; j < handles.length; j++){
			mPush();
			mTranslate(handles[j]._data[1][0],handles[j]._data[2][0],handles[j]._data[3][0]);
			sphere(diameter);
			mPop();
		}
		
	}
	
	function getEnergy(state) {
		E_Weight = 0;
		E_Length = 0;
		E_Coherence = 0;
		E_QuatNorm = 0;
		E_BendingTorsion = 0;
		E_Handles = 0;

		for(let i = 0; i < n; i++){
			E_Weight = E_Weight + math.squeeze(math.multiply(math.multiply(m, math.transpose(g)), math.squeeze(math.subset(state, math.index(math.range(i*7,i*7+3),0))))); //-m * g * x_i
			if (i < n-1){
				L[i] = math.norm(math.squeeze(math.subtract(math.subset(state, math.index(math.range(i*7+7,i*7+10),0)), math.subset(state, math.index(math.range(i*7,i*7+3),0))))); //L[i] = L[i] = |x[i+1]-x[i]|
				u[i] = math.divide(math.subtract(math.subset(state, math.index(math.range(i*7+7,i*7+10),0)), math.subset(state, math.index(math.range(i*7,i*7+3),0))),L[i]);
				r[i] = math.matrix([[2*(state._data[i*7+4][0]*state._data[i*7+5][0]-state._data[i*7+3][0]*state._data[i*7+6][0])] , [math.pow(state._data[i*7+3][0],2)-math.pow(state._data[i*7+4][0],2)+math.pow(state._data[i*7+5][0],2)-math.pow(state._data[i*7+6][0],2)] , [2*(state._data[i*7+3][0]*state._data[i*7+4][0]+state._data[i*7+5][0]*state._data[i*7+6][0])]]);

				E_Length = E_Length + 0.5 * k_Length * math.pow(L[i]-L_ref,2) + f_iI[i] * (L[i]-L_ref);
				E_Coherence = E_Coherence + 0.5 * k_Coh * math.pow(math.norm(math.squeeze(math.subtract(u[i], r[i]))),2) + math.squeeze(math.multiply(math.transpose(math.subtract(u[i], r[i])),f_cI[i]));
				E_QuatNorm = E_QuatNorm + 0.5 * k_QuatNorm * math.pow((1/math.norm(math.squeeze(math.subset(state, math.index(math.range(i*7+3,i*7+7),0)))))-1,2) + f_qI[i] * ((1/math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))))-1);
			}
			if (i < n - 2){
				omega[i] = math.subset(math.multiply(2/L_ref, math.matrix([[state._data[i*7+3][0]*state._data[i*7+10][0] + state._data[i*7+4][0]*state._data[i*7+11][0] +  state._data[i*7+5][0]*state._data[i*7+12][0] +  state._data[i*7+6][0]*state._data[i*7+13][0]],[state._data[i*7+3][0]*state._data[i*7+11][0] - state._data[i*7+4][0]*state._data[i*7+10][0] -  state._data[i*7+5][0]*state._data[i*7+13][0] +  state._data[i*7+6][0]*state._data[i*7+12][0]],[state._data[i*7+3][0]*state._data[i*7+12][0] - state._data[i*7+5][0]*state._data[i*7+10][0] -  state._data[i*7+6][0]*state._data[i*7+11][0] +  state._data[i*7+4][0]*state._data[i*7+13][0]],[state._data[i*7+3][0]*state._data[i*7+13][0] - state._data[i*7+6][0]*state._data[i*7+10][0] -  state._data[i*7+4][0]*state._data[i*7+12][0] +  state._data[i*7+5][0]*state._data[i*7+11][0]]])), math.index(math.range(1,4),0));
				E_BendingTorsion = E_BendingTorsion + 0.5 * math.squeeze(math.multiply(math.multiply(math.transpose(omega[i]), A), omega[i]));
			}
		}
		
		for(let j = 0; j < handles.length; j++){
			let i = math.floor(handles[j]._data[0][0] * (n-1));
			let r = (((L_ref * (n-1)) * handles[j]._data[0]) - (i * L_ref))/L_ref;
			E_Handles = E_Handles + k_Handles * (math.pow((1-r) * state._data[i*7][0] + r * state._data[i*7+7][0] - handles[j]._data[1][0],2) + math.pow((1-r) * state._data[i*7+1][0] + r * state._data[i*7+8][0] - handles[j]._data[2][0],2) + math.pow((1 - r) * state._data[i*7+2][0] + r * state._data[i*7+9][0] - handles[j]._data[3][0],2) + math.pow(state._data[i*7+3][0] - handles[j]._data[4][0],2)  + math.pow(state._data[i*7+4][0] - handles[j]._data[5][0],2) + math.pow(state._data[i*7+5][0] - handles[j]._data[6][0],2) + math.pow(state._data[i*7+6][0] - handles[j]._data[7][0],2));
		}
		return E_Weight + E_Length + E_QuatNorm + E_Coherence + E_BendingTorsion + E_Handles;
	}
	
	function updateReusableVar(){
		for(let i = 0; i < n-1; i++){ //update L_i, u_i, r_i, omega_i
			L[i] = math.norm(math.squeeze(math.subtract(math.subset(X, math.index(math.range(i*7+7,i*7+10),0)), math.subset(X, math.index(math.range(i*7,i*7+3),0))))); //L[i] = L[i] = |x[i+1]-x[i]|
			u[i] = math.divide(math.subtract(math.subset(X, math.index(math.range(i*7+7,i*7+10),0)), math.subset(X, math.index(math.range(i*7,i*7+3),0))),L[i]);
			r[i] = math.matrix([[2*(X._data[i*7+4][0]*X._data[i*7+5][0]-X._data[i*7+3][0]*X._data[i*7+6][0])] , [math.pow(X._data[i*7+3][0],2)-math.pow(X._data[i*7+4][0],2)+math.pow(X._data[i*7+5][0],2)-math.pow(X._data[i*7+6][0],2)] , [2*(X._data[i*7+3][0]*X._data[i*7+4][0]+X._data[i*7+5][0]*X._data[i*7+6][0])]]);
			if (i < n - 2){
				omega[i] = math.subset(math.multiply(2/L_ref, math.matrix([[X._data[i*7+3][0]*X._data[i*7+10][0] + X._data[i*7+4][0]*X._data[i*7+11][0] +  X._data[i*7+5][0]*X._data[i*7+12][0] +  X._data[i*7+6][0]*X._data[i*7+13][0]],[X._data[i*7+3][0]*X._data[i*7+11][0] - X._data[i*7+4][0]*X._data[i*7+10][0] -  X._data[i*7+5][0]*X._data[i*7+13][0] +  X._data[i*7+6][0]*X._data[i*7+12][0]],[X._data[i*7+3][0]*X._data[i*7+12][0] - X._data[i*7+5][0]*X._data[i*7+10][0] -  X._data[i*7+6][0]*X._data[i*7+11][0] +  X._data[i*7+4][0]*X._data[i*7+13][0]],[X._data[i*7+3][0]*X._data[i*7+13][0] - X._data[i*7+6][0]*X._data[i*7+10][0] -  X._data[i*7+4][0]*X._data[i*7+12][0] +  X._data[i*7+5][0]*X._data[i*7+11][0]]])), math.index(math.range(1,4),0));
			};
		}
	}
	
	//Energy functions
	function getEweight(){
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let i = 0; i < n; i++){ //acts on each point
			E._data[i*2][0] = math.squeeze(math.multiply(math.multiply(m, math.transpose(g)), math.squeeze(math.subset(X, math.index(math.range(i*7,i*7+3),0))))); //-m * g * x_i
		}
		return E;
	};
	
	function getElength(){ //ok
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on each interval
			E._data[(i*2) + 1][0] = 0.5 * k_Length * math.pow(L[i]-L_ref,2);
		}
		return E;
	}
	
	function getEIlength(){
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on each interval
			E._data[(i*2) + 1][0] = f_iI[i] * (L[i]-L_ref);
		}
		return E;
	}
	
	function getEcoherence(){
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on each interval
			E._data[(i*2) + 1][0] = 0.5 * k_Coh * math.pow(math.norm(math.squeeze(math.subtract(u[i], r[i]))),2);
		}
		return E;
	}
	
	function getEIcoherence(){
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on each interval
		    E._data[(i*2) + 1][0] = math.squeeze(math.multiply(math.transpose(math.subtract(u[i], r[i])),f_cI[i]));
		}
		return E;
	}
	
	function getEquatnorm(){
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on each interval
			E._data[(i*2) + 1][0] = 0.5 * k_QuatNorm * math.pow((1/math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))))-1,2);
		}
		return E;
	}
	
	function getEIquatnorm(){
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on each interval
			E._data[(i*2) + 1][0] =  f_qI[i] * ((1/math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))))-1);
		}
		return E;
	}		
	
	function getEbendingtorsion() {
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let i = 0; i < n-2; i++){ //acts on the points between start and finish
			E._data[(i+1)*2][0] = 0.5 * math.squeeze(math.multiply(math.multiply(math.transpose(omega[i]), A), omega[i]));
		}
		return E;
	};

	function getEbending() {
		updateReusableVar();
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let i = 0; i < n-2; i++){ //acts on the points between start and finish
			E._data[(i+1)*2][0] = 0.5 * math.squeeze(math.multiply(math.multiply(math.transpose(omega[i]), math.matrix([[B_kappa,0,0],[0,0,0],[0,0,B_kappa]])), omega[i]));
		}
		return E;
	};
	
	function getEtorsion() {
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let i = 0; i < n-2; i++){ //acts on the points between start and finish
			E._data[(i+1)*2][0] = 0.5 * math.squeeze(math.multiply(math.multiply(math.transpose(omega[i]), math.matrix([[0,0,0],[0,B_tau,0],[0,0,0]])), omega[i]));
		}
		return E;
	};

	function getEhandles(){
		let E = math.zeros((n * 2 - 1),1);// for each point and each interval
		for(let j = 0; j < handles.length; j++){
			let i = math.floor(handles[j]._data[0][0] * (n-1)); //act on the interval i
			let r = (((L_ref * (n-1)) * handles[j]._data[0]) - (i * L_ref))/L_ref;
			E._data[(i*2) + 1][0] = k_Handles * (math.pow((1-r) * X._data[i*7][0] + r * X._data[i*7+7][0] - handles[j]._data[1][0],2) + math.pow((1-r) * X._data[i*7+1][0] + r * X._data[i*7+8][0] - handles[j]._data[2][0],2) + math.pow((1 - r) * X._data[i*7+2][0] + r * X._data[i*7+9][0] - handles[j]._data[3][0],2) + math.pow(X._data[i*7+3][0] - handles[j]._data[4][0],2)  + math.pow(X._data[i*7+4][0] - handles[j]._data[5][0],2) + math.pow(X._data[i*7+5][0] - handles[j]._data[6][0],2) + math.pow(X._data[i*7+6][0] - handles[j]._data[7][0],2));
		}
		return E;
	};
	
	//Force functions
	function getFweight() {
		let F = math.zeros((7 * n) - 4,1); // for each point and each interval
		let F_Weight = math.multiply(m, g);
		for(let i = 0; i < n; i++){ //acts on each point
			F = math.subset(F, math.index(math.range(i*7,i*7 + 3),0), F_Weight);
		};
		return F;
	}

	function getFlength() { //ok
		let F = math.zeros((7 * n) - 4,1); // for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on the points around each interval
			let F_Length = math.zeros(10,1); //temporal buffer in which the intermediate results are stored
			F_Length = math.subset(F_Length, math.index(math.range(0,3),0),math.multiply(k_Length * (L[i]-L_ref),u[i]));
			F_Length = math.subset(F_Length, math.index(math.range(7,10),0),math.multiply(math.subset(F_Length, math.index(math.range(0,3),0)),-1));
			F = math.subset(F, math.index(math.range(i*7,i*7 + 10),0), math.add(math.subset(F, math.index(math.range(i*7,i*7 +10),0)), F_Length));
		}
		return F;
	}
	
	function getFIlength() {
		let F = math.zeros((7 * n) - 4,1); // for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on the points around each interval
			let F_Length = math.zeros(10,1); //temporal buffer in which the intermediate results are stored
			F_Length = math.subset(F_Length, math.index(math.range(0,3),0),math.multiply(f_iI[i],u[i]));
			F_Length = math.subset(F_Length, math.index(math.range(7,10),0),math.multiply(math.subset(F_Length, math.index(math.range(0,3),0)),-1));
			F = math.subset(F, math.index(math.range(i*7,i*7 + 10),0), math.add(math.subset(F, math.index(math.range(i*7,i*7 +10),0)), F_Length));
		}
		return F;
	}
	
	function getFcoherence(){
		let F = math.zeros((7 * n) - 4,1); // for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on the points and the intervals
			let F_Coherence = math.zeros(10,1); //temporal buffer in which the intermediate results are stored
			F_Coherence._data[0][0] = k_Coh*((X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			F_Coherence._data[1][0] = k_Coh*((X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			F_Coherence._data[2][0] = k_Coh*((X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			F_Coherence._data[3][0] = -2*k_Coh*(X._data[i*7+3][0]*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) + X._data[i*7+4][0]*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) - X._data[i*7+6][0]*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			F_Coherence._data[4][0] = -2*k_Coh*(X._data[i*7+3][0]*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) - X._data[i*7+4][0]*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) + X._data[i*7+5][0]*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			F_Coherence._data[5][0] = -2*k_Coh*(X._data[i*7+4][0]*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) + X._data[i*7+5][0]*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) + X._data[i*7+6][0]*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			F_Coherence._data[6][0] = 2*k_Coh*(X._data[i*7+3][0]*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) - X._data[i*7+5][0]*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) + X._data[i*7+6][0]*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			F_Coherence = math.subset(F_Coherence, math.index(math.range(7,10),0),math.multiply(math.subset(F_Coherence, math.index(math.range(0,3),0)),-1));
			F = math.subset(F, math.index(math.range(i*7,i*7 + 10),0), math.add(math.subset(F, math.index(math.range(i*7,i*7 +10),0)), F_Coherence));
		}
		return F;
	}

	function getFIcoherence(){
		let F = math.zeros((7 * n) - 4,1); // for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on the points and the intervals
			let F_Coherence = math.zeros(10,1); //temporal buffer in which the intermediate results are stored
			F_Coherence._data[0][0] = math.multiply(math.squeeze(math.multiply(math.transpose(math.subtract(math.multiply(u[i], u[i]._data[0][0]/L[i]),[[1/L[i]], [0], [0]])),f_cI[i])),-1);
			F_Coherence._data[1][0] = math.multiply(math.squeeze(math.multiply(math.transpose(math.subtract(math.multiply(u[i], u[i]._data[1][0]/L[i]),[[0], [1/L[i]], [0]])),f_cI[i])),-1);
			F_Coherence._data[2][0] = math.multiply(math.squeeze(math.multiply(math.transpose(math.subtract(math.multiply(u[i], u[i]._data[2][0]/L[i]),[[0], [0], [1/L[i]]])),f_cI[i])),-1);

			F_Coherence._data[3][0] = math.multiply(math.squeeze(math.multiply([[ 2*X._data[i*7+6][0],-2*X._data[i*7+3][0],-2*X._data[i*7+4][0]]],f_cI[i])),-1);
			F_Coherence._data[4][0] = math.multiply(math.squeeze(math.multiply([[-2*X._data[i*7+5][0], 2*X._data[i*7+4][0],-2*X._data[i*7+3][0]]],f_cI[i])),-1);
			F_Coherence._data[5][0] = math.multiply(math.squeeze(math.multiply([[-2*X._data[i*7+4][0],-2*X._data[i*7+5][0],-2*X._data[i*7+6][0]]],f_cI[i])),-1);
			F_Coherence._data[6][0] = math.multiply(math.squeeze(math.multiply([[ 2*X._data[i*7+3][0], 2*X._data[i*7+6][0],-2*X._data[i*7+5][0]]],f_cI[i])),-1);

			F_Coherence = math.subset(F_Coherence, math.index(math.range(7,10),0),math.multiply(math.subset(F_Coherence, math.index(math.range(0,3),0)),-1));
			F = math.subset(F, math.index(math.range(i*7,i*7 + 10),0), math.add(math.subset(F, math.index(math.range(i*7,i*7 +10),0)), F_Coherence));
		}
		return F;
	}

	function getFquatnorm(){
		let F = math.zeros((7 * n) - 4,1); // for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on each interval
			let F_QuatNorm = math.zeros(7,1); //temporal buffer in which the intermediate results are stored
			F_QuatNorm = math.subset(F_QuatNorm, math.index(math.range(3,7),0), math.multiply(k_QuatNorm * ((1/math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))))-1) / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),3), math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))));
			F = math.subset(F, math.index(math.range(i*7,i*7 + 7),0), F_QuatNorm);
		}
		return F;
	}
	
	function getFIquatnorm(){
		let F = math.zeros((7 * n) - 4,1); // for each point and each interval
		for(let i = 0; i < n - 1; i++){ //acts on each interval
			let F_QuatNorm = math.zeros(7,1); //temporal buffer in which the intermediate results are stored
			F_QuatNorm = math.subset(F_QuatNorm, math.index(math.range(3,7),0), math.multiply(-f_qI[i] * (1/math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),3)), math.subset(X, math.index(math.range(i*7+3,i*7+7),0))));
			F = math.subset(F, math.index(math.range(i*7,i*7 + 7),0), F_QuatNorm);
		}
		return F;
	}
	
	function getFbendingtorsion(){
		let F = math.zeros((7 * n) - 4,1); // for each point and each interval
		for(let i = 0; i < n-2; i++){ //acts on each interval
			let B = math.matrix([[ X._data[i*7+11][0],-X._data[i*7+10][0],-X._data[i*7+13][0], X._data[i*7+12][0]]
								,[ X._data[i*7+12][0], X._data[i*7+13][0],-X._data[i*7+10][0],-X._data[i*7+11][0]]
								,[ X._data[i*7+13][0],-X._data[i*7+12][0], X._data[i*7+11][0],-X._data[i*7+10][0]]]);
								
			let C = math.matrix([[-X._data[i*7+ 4][0], X._data[i*7+ 3][0], X._data[i*7+ 6][0],-X._data[i*7+ 5][0]]
								,[-X._data[i*7+ 5][0],-X._data[i*7+ 6][0], X._data[i*7+ 3][0], X._data[i*7+ 4][0]]
								,[-X._data[i*7+ 6][0], X._data[i*7+ 5][0],-X._data[i*7+ 4][0], X._data[i*7+ 3][0]]]);

			let F_BendingTorsion = math.zeros(14,1); //temporal buffer in which the intermediate results are stored
			F_BendingTorsion = math.subset(F_BendingTorsion, math.index(math.range( 3, 7),0),math.squeeze(math.multiply(-2/L_ref,math.multiply(math.transpose(B),math.multiply(A, omega[i])))));
			F_BendingTorsion = math.subset(F_BendingTorsion, math.index(math.range(10,14),0),math.squeeze(math.multiply(-2/L_ref,math.multiply(math.transpose(C),math.multiply(A, omega[i])))));
			F = math.subset(F, math.index(math.range(i*7,i*7 + 14),0), math.add(math.subset(F, math.index(math.range(i*7,i*7 +14),0)), F_BendingTorsion));
		}
		return F;
	};
	
	function getFhandles(){
		let F = math.zeros((7 * n) - 4,1); // for each point and each interval
		for(let j = 0; j < handles.length; j++){
			let i = math.floor(handles[j]._data[0][0] * (n-1)); //act on the interval i
			let r = (((L_ref * (n-1)) * handles[j]._data[0]) - (i * L_ref))/L_ref;
			let F_Handles = math.zeros(10,1); //temporal buffer in which the intermediate results are stored
			F_Handles = math.subset(F_Handles, math.index(math.range( 0, 3),0), math.multiply(k_Handles * (1-r), math.subtract(math.add(math.multiply((1-r),math.subset(X, math.index(math.range(i*7,i*7+3),0))),math.multiply(r,math.subset(X, math.index(math.range(i*7+7,i*7+10),0)))),	math.subset(handles[j],math.index(math.range(1,4),0)))));
			F_Handles = math.subset(F_Handles, math.index(math.range( 3, 7),0),math.multiply(k_Handles, math.subtract(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)),math.subset(handles[j],math.index(math.range(4,8),0)))));
			F_Handles = math.subset(F_Handles, math.index(math.range( 7,10),0),math.multiply(-(r/(r-1)),math.subset(F_Handles, math.index(math.range( 0, 3),0))));
			F = math.subset(F, math.index(math.range(i*7,i*7 + 10),0), math.add(math.subset(F, math.index(math.range(i*7,i*7 +10),0)), F_Handles));
		}
		return F;
	};

	//Hessian functions
	function getHlength(){
		let H = math.zeros((7 * n) - 4, (7 * n) - 4); // for each point and each interval in 2D
		for(let i = 0; i < n - 1; i++){ //acts on the points around each interval
			let H_Length = math.zeros(10,10); //temporal buffer in which the intermediate results are stored
			H_Length._data[1][0] = k_Length*(L_ref - math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + k_Length*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])/(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2));
			H_Length._data[2][0] = k_Length*(L_ref - math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + k_Length*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2));
			H_Length._data[2][1] = k_Length*(L_ref - math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + k_Length*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2));
			H_Length = math.add(H_Length,math.transpose(H_Length));
			H_Length._data[0][0] = k_Length*(L_ref - math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))*math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - k_Length*(L_ref - math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)) + k_Length*math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)/(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2));
			H_Length._data[1][1] = k_Length*(L_ref - math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))*math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - k_Length*(L_ref - math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)) + k_Length*math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2));
			H_Length._data[2][2] = k_Length*(L_ref - math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - k_Length*(L_ref - math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)) + k_Length*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2));
			H_Length = math.subset(H_Length, math.index(math.range(0,3), math.range(7,10)), math.multiply(math.subset(H_Length, math.index(math.range(0,3), math.range(0,3))),-1));
			H_Length = math.subset(H_Length, math.index(math.range(7,10), math.range(0,10)), math.multiply(math.subset(H_Length, math.index(math.range(0,3), math.range(0,10))),-1));
			H = math.subset(H, math.index(math.range(i*7,i*7 + 10), math.range(i*7,i*7 + 10)), math.add(math.subset(H, math.index(math.range(i*7,i*7 + 10), math.range(i*7,i*7 + 10))), H_Length));
		}
		return H;
	}
	
	function getHIlength(){
		let H = math.zeros((7 * n) - 4, (7 * n) - 4); // for each point and each interval in 2D
		for(let i = 0; i < n - 1; i++){ //acts on the points around each interval
			let H_Length = math.zeros(10,10); //temporal buffer in which the intermediate results are stored
			H_Length._data[1][0] = f_iI[i] * (X._data[i*7+1][0] - X._data[i*7+8][0]) * (X._data[i*7+0][0] - X._data[i*7+7][0], 2) / math.pow(L[i],3)
			H_Length._data[2][0] = f_iI[i] * (X._data[i*7+2][0] - X._data[i*7+9][0]) * (X._data[i*7+0][0] - X._data[i*7+7][0], 2) / math.pow(L[i],3)
			H_Length._data[2][1] = f_iI[i] * (X._data[i*7+2][0] - X._data[i*7+9][0]) * (X._data[i*7+1][0] - X._data[i*7+8][0], 2) / math.pow(L[i],3)
			H_Length = math.add(H_Length,math.transpose(H_Length));
			H_Length._data[0][0] = f_iI[i] * ((math.pow(X._data[i*7+0][0] - X._data[i*7+7][0],2) /  math.pow(L[i],3)) - (1 / L[i]))
			H_Length._data[1][1] = f_iI[i] * ((math.pow(X._data[i*7+1][0] - X._data[i*7+8][0],2) /  math.pow(L[i],3)) - (1 / L[i]))
			H_Length._data[2][2] = f_iI[i] * ((math.pow(X._data[i*7+2][0] - X._data[i*7+9][0],2) /  math.pow(L[i],3)) - (1 / L[i]))
			H_Length = math.subset(H_Length, math.index(math.range(0,3), math.range(7,10)), math.multiply(math.subset(H_Length, math.index(math.range(0,3), math.range(0,3))),-1));
			H_Length = math.subset(H_Length, math.index(math.range(7,10), math.range(0,10)), math.multiply(math.subset(H_Length, math.index(math.range(0,3), math.range(0,10))),-1));
			H = math.subset(H, math.index(math.range(i*7,i*7 + 10), math.range(i*7,i*7 + 10)), math.add(math.subset(H, math.index(math.range(i*7,i*7 + 10), math.range(i*7,i*7 + 10))), H_Length));
		}
		return H;
	}
	
	function getHcoherence(){
		let H = math.zeros((7 * n) - 4, (7 * n) - 4); // for each point and each interval in 2D
		for(let i = 0; i < n - 1; i++){ //acts on the points and the intervals
			let H_Coherence = math.zeros(10,10); //temporal buffer in which the intermediate results are stored
			//lower left corner of upper left square (dxdx)
			H_Coherence._data[1][0] = k_Coh*(3*math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)*(X._data[i*7+1][0] - X._data[i*7+8][0])*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) + (X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3) + 3*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) + (X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])*(math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - (X._data[i*7+1][0] - X._data[i*7+8][0])*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (3*(X._data[i*7][0] - X._data[i*7+7][0])*math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7][0] - X._data[i*7+7][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0))*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			H_Coherence._data[2][0] = k_Coh*(3*math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)*(X._data[i*7+2][0] - X._data[i*7+9][0])*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) + (X._data[i*7][0] - X._data[i*7+7][0])*math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3) + 3*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) + (X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - (X._data[i*7+2][0] - X._data[i*7+9][0])*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (3*(X._data[i*7][0] - X._data[i*7+7][0])*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7][0] - X._data[i*7+7][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0))*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			H_Coherence._data[2][1] = k_Coh*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3) + 3*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) + 3*math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)*(X._data[i*7+2][0] - X._data[i*7+9][0])*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) + (X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])*(math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - (X._data[i*7+2][0] - X._data[i*7+9][0])*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + (3*(X._data[i*7+1][0] - X._data[i*7+8][0])*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0))*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			//lower left corner of center square (dqdq)
			H_Coherence._data[4][3] = 2*k_Coh*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)));
			H_Coherence._data[5][3] = 4*k_Coh*X._data[i*7+3][0]*X._data[i*7+5][0];
			H_Coherence._data[6][3] = 2*k_Coh*(2*X._data[i*7+3][0]*X._data[i*7+6][0] - (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)));;
			H_Coherence._data[5][4] = 2*k_Coh*(2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)));
			H_Coherence._data[6][4] = 4*k_Coh*X._data[i*7+4][0]*X._data[i*7+6][0];
			H_Coherence._data[6][5] = 2*k_Coh*(2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)));
			//elements from center left rectangle (dxdq)
			H_Coherence._data[3][0] = 2*k_Coh*(-X._data[i*7+3][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - X._data[i*7+4][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+6][0]*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			H_Coherence._data[3][1] = -2*k_Coh*(X._data[i*7+3][0]*(math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) + X._data[i*7+4][0]*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - X._data[i*7+6][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0));
			H_Coherence._data[3][2] = -2*k_Coh*(X._data[i*7+3][0]*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+4][0]*(math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) - X._data[i*7+6][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0));
			H_Coherence._data[4][0] = -2*k_Coh*(X._data[i*7+3][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - X._data[i*7+4][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+5][0]*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			H_Coherence._data[4][1] = 2*k_Coh*(-X._data[i*7+3][0]*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+4][0]*(math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) - X._data[i*7+5][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0));
			H_Coherence._data[4][2] = -2*k_Coh*(X._data[i*7+3][0]*(math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) - X._data[i*7+4][0]*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+5][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0));
			H_Coherence._data[5][0] = -2*k_Coh*(X._data[i*7+4][0]*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) + X._data[i*7+5][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+6][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0));
			H_Coherence._data[5][1] = -2*k_Coh*(X._data[i*7+4][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+5][0]*(math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) + X._data[i*7+6][0]*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0));
			H_Coherence._data[5][2] = -2*k_Coh*(X._data[i*7+4][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+5][0]*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+6][0]*(math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			H_Coherence._data[6][0] = 2*k_Coh*(X._data[i*7+3][0]*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) - X._data[i*7+5][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+6][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0));
			H_Coherence._data[6][1] = 2*k_Coh*(X._data[i*7+3][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - X._data[i*7+5][0]*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+6][0]*(math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			H_Coherence._data[6][2] = -2*k_Coh*(-X._data[i*7+3][0]*(X._data[i*7][0] - X._data[i*7+7][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + X._data[i*7+5][0]*(math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))) - X._data[i*7+6][0]*(X._data[i*7+1][0] - X._data[i*7+8][0])*(X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0));
			//transpose + add
			H_Coherence = math.add(H_Coherence,math.transpose(H_Coherence));
			//diagonal
			H_Coherence._data[0][0] = k_Coh*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)*math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3) + 3*math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)*(X._data[i*7+1][0] - X._data[i*7+8][0])*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) + math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3) + 3*math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)*(X._data[i*7+2][0] - X._data[i*7+9][0])*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7+1][0] - X._data[i*7+8][0])*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - (X._data[i*7+2][0] - X._data[i*7+9][0])*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)), 2) + 3*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 3)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7][0] - X._data[i*7+7][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0))*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			H_Coherence._data[1][1] = k_Coh*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)*math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3) + 3*(X._data[i*7][0] - X._data[i*7+7][0])*math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7][0] - X._data[i*7+7][0])*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3) + 3*math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)*(X._data[i*7+2][0] - X._data[i*7+9][0])*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7+2][0] - X._data[i*7+9][0])*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + math.pow(math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)), 2) + 3*(math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 3)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7+1][0] - X._data[i*7+8][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0))*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			H_Coherence._data[2][2] = k_Coh*(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2)*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3) + 3*(X._data[i*7][0] - X._data[i*7+7][0])*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7][0] - X._data[i*7+7][0])*(-2*X._data[i*7+3][0]*X._data[i*7+6][0] + 2*X._data[i*7+4][0]*X._data[i*7+5][0] + (X._data[i*7][0] - X._data[i*7+7][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2)*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3) + 3*(X._data[i*7+1][0] - X._data[i*7+8][0])*math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7+1][0] - X._data[i*7+8][0])*(math.pow(X._data[i*7+3][0], 2) - math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) - math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)))/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) + math.pow(math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0) - 1/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)), 2) + 3*(math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 3)/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 5.0/2.0) - (X._data[i*7+2][0] - X._data[i*7+9][0])/math.pow(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2), 3.0/2.0))*(2*X._data[i*7+3][0]*X._data[i*7+4][0] + 2*X._data[i*7+5][0]*X._data[i*7+6][0] + (X._data[i*7+2][0] - X._data[i*7+9][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2))));
			H_Coherence._data[3][3] = 2*k_Coh*(3*math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)));
			H_Coherence._data[4][4] = 2*k_Coh*(math.pow(X._data[i*7+3][0], 2) + 3*math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2) - (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)));
			H_Coherence._data[5][5] = 2*k_Coh*(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + 3*math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2) + (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)));
			H_Coherence._data[6][6] = 2*k_Coh*(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + 3*math.pow(X._data[i*7+6][0], 2) - (X._data[i*7+1][0] - X._data[i*7+8][0])/math.sqrt(math.pow(X._data[i*7][0] - X._data[i*7+7][0], 2) + math.pow(X._data[i*7+1][0] - X._data[i*7+8][0], 2) + math.pow(X._data[i*7+2][0] - X._data[i*7+9][0], 2)));
			//copy & copy + multiply
			H_Coherence = math.subset(H_Coherence, math.index(math.range(0,3), math.range(7,10)), math.multiply(math.subset(H_Coherence, math.index(math.range(0,3), math.range(0,3))),-1));
			H_Coherence = math.subset(H_Coherence, math.index(math.range(7,10), math.range(0,10)), math.multiply(math.subset(H_Coherence, math.index(math.range(0,3), math.range(0,10))),-1));
			H_Coherence = math.subset(H_Coherence, math.index(math.range(3,7), math.range(7,10)), math.multiply(math.subset(H_Coherence, math.index(math.range(3,7), math.range(0,3))),-1));
			H = math.subset(H, math.index(math.range(i*7,i*7 + 10), math.range(i*7,i*7 + 10)), math.add(math.subset(H, math.index(math.range(i*7,i*7 + 10), math.range(i*7,i*7 + 10))), H_Coherence));
		}
		return H;
	}
	
	function getHIcoherence(){
		let H = math.zeros((7 * n) - 4, (7 * n) - 4); // for each point and each interval in 2D
		for(let i = 0; i < n - 1; i++){ //acts on the points and the intervals
			let H_Coherence = math.zeros(10,10); //temporal buffer in which the intermediate results are stored
			//lower left corner of upper left square (dxdx)
			H_Coherence._data[1][0] = math.squeeze(math.multiply(math.transpose(math.subtract(math.subtract(math.multiply(u[i], 3 * u[i]._data[1][0] * u[i]._data[0][0] / (L[i] * L[i])),[[u[i]._data[1][0]/(L[i]*L[i])], [0], [0]]),[[0], [u[i]._data[0][0]/(L[i]*L[i])], [0]])),f_cI[i]));
			H_Coherence._data[2][0] = math.squeeze(math.multiply(math.transpose(math.subtract(math.subtract(math.multiply(u[i], 3 * u[i]._data[2][0] * u[i]._data[0][0] / (L[i] * L[i])),[[u[i]._data[2][0]/(L[i]*L[i])], [0], [0]]),[[0], [0], [u[i]._data[0][0]/(L[i]*L[i])]])),f_cI[i]));
			H_Coherence._data[2][1] = math.squeeze(math.multiply(math.transpose(math.subtract(math.subtract(math.multiply(u[i], 3 * u[i]._data[2][0] * u[i]._data[1][0] / (L[i] * L[i])),[[0], [u[i]._data[2][0]/(L[i]*L[i])], [0]]),[[0], [0], [u[i]._data[1][0]/(L[i]*L[i])]])),f_cI[i]));
			//lower left corner of center square (dqdq)
			H_Coherence._data[4][3] = math.squeeze(math.multiply(math.transpose([[ 0],[ 0],[-2]]),f_cI[i]));
			H_Coherence._data[5][3] = math.squeeze(math.multiply(math.transpose([[ 0],[ 0],[ 0]]),f_cI[i]));
			H_Coherence._data[6][3] = math.squeeze(math.multiply(math.transpose([[ 2],[ 0],[ 0]]),f_cI[i]));
			H_Coherence._data[5][4] = math.squeeze(math.multiply(math.transpose([[-2],[ 0],[ 0]]),f_cI[i]));
			H_Coherence._data[6][4] = math.squeeze(math.multiply(math.transpose([[ 0],[ 0],[ 0]]),f_cI[i]));
			H_Coherence._data[6][5] = math.squeeze(math.multiply(math.transpose([[ 0],[ 0],[-2]]),f_cI[i]));
			//elements from center left rectangle (dxdq) are non existent (no mixed types)			
			//transpose + add
			H_Coherence = math.add(H_Coherence,math.transpose(H_Coherence));		
			//diagonal
			H_Coherence._data[0][0] = math.squeeze(math.multiply(math.transpose(math.subtract(math.subtract(math.multiply(u[i], 3 * u[i]._data[0][0] * u[i]._data[0][0] / (L[i] * L[i])),	math.multiply(u[i], 1 / (L[i] * L[i]))),[[2 * u[i]._data[0][0]/(L[i]*L[i])], [0], [0]])),f_cI[i]));
			H_Coherence._data[1][1] = math.squeeze(math.multiply(math.transpose(math.subtract(math.subtract(math.multiply(u[i], 3 * u[i]._data[1][0] * u[i]._data[1][0] / (L[i] * L[i])),	math.multiply(u[i], 1 / (L[i] * L[i]))),[[0], [2 * u[i]._data[1][0]/(L[i]*L[i])], [0]])),f_cI[i]));
			H_Coherence._data[2][2] = math.squeeze(math.multiply(math.transpose(math.subtract(math.subtract(math.multiply(u[i], 3 * u[i]._data[2][0] * u[i]._data[2][0] / (L[i] * L[i])),	math.multiply(u[i], 1 / (L[i] * L[i]))),[[0], [0], [2 * u[i]._data[2][0]/(L[i]*L[i])]])),f_cI[i]));
			H_Coherence._data[3][3] = math.squeeze(math.multiply(math.transpose([[ 0],[-2],[ 0]]),f_cI[i]));
			H_Coherence._data[4][4] = math.squeeze(math.multiply(math.transpose([[ 0],[ 2],[ 0]]),f_cI[i]));
			H_Coherence._data[5][5] = math.squeeze(math.multiply(math.transpose([[ 0],[-2],[ 0]]),f_cI[i]));
			H_Coherence._data[6][6] = math.squeeze(math.multiply(math.transpose([[ 0],[ 2],[ 0]]),f_cI[i]));
			//copy & copy + multiply
			H_Coherence = math.subset(H_Coherence, math.index(math.range(0,3), math.range(7,10)), math.multiply(math.subset(H_Coherence, math.index(math.range(0,3), math.range(0,3))),-1));
			H_Coherence = math.subset(H_Coherence, math.index(math.range(7,10), math.range(0,10)), math.multiply(math.subset(H_Coherence, math.index(math.range(0,3), math.range(0,10))),-1));
			H_Coherence = math.subset(H_Coherence, math.index(math.range(3,7), math.range(7,10)), math.multiply(math.subset(H_Coherence, math.index(math.range(3,7), math.range(0,3))),-1));
			H = math.subset(H, math.index(math.range(i*7,i*7 + 10), math.range(i*7,i*7 + 10)), math.add(math.subset(H, math.index(math.range(i*7,i*7 + 10), math.range(i*7,i*7 + 10))), H_Coherence));
		}
		return H;
	}
	
	function getHquatnorm(){
		let H = math.zeros((7 * n) - 4, (7 * n) - 4); // for each point and each interval in 2D
		for(let i = 0; i < n - 1; i++){ //acts on the points and the intervals
			let H_QuatNorm = math.zeros(7,7); //temporal buffer in which the intermediate results are stored
			H_QuatNorm._data[3][4] = 3*k_QuatNorm*X._data[i*7+3][0]*X._data[i*7+4][0]*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 5.0/2.0) + k_QuatNorm*X._data[i*7+3][0]*X._data[i*7+4][0]/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3);
			H_QuatNorm._data[3][5] = 3*k_QuatNorm*X._data[i*7+3][0]*X._data[i*7+4][0]*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 5.0/2.0) + k_QuatNorm*X._data[i*7+3][0]*X._data[i*7+4][0]/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3);
			H_QuatNorm._data[3][6] = 3*k_QuatNorm*X._data[i*7+3][0]*X._data[i*7+6][0]*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 5.0/2.0) + k_QuatNorm*X._data[i*7+3][0]*X._data[i*7+6][0]/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3);
			H_QuatNorm._data[4][5] = 3*k_QuatNorm*X._data[i*7+4][0]*X._data[i*7+5][0]*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 5.0/2.0) + k_QuatNorm*X._data[i*7+4][0]*X._data[i*7+5][0]/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3);
			H_QuatNorm._data[4][6] = 3*k_QuatNorm*X._data[i*7+4][0]*X._data[i*7+6][0]*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 5.0/2.0) + k_QuatNorm*X._data[i*7+4][0]*X._data[i*7+6][0]/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3);
			H_QuatNorm._data[5][6] = 3*k_QuatNorm*X._data[i*7+5][0]*X._data[i*7+6][0]*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 5.0/2.0) + k_QuatNorm*X._data[i*7+5][0]*X._data[i*7+6][0]/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3);
			H_QuatNorm = math.add(H_QuatNorm,math.transpose(H_QuatNorm));
			H_QuatNorm._data[3][3] = 3*k_QuatNorm*math.pow(X._data[i*7+3][0], 2)*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 5.0/2.0) + k_QuatNorm*math.pow(X._data[i*7+3][0], 2)/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3) - k_QuatNorm*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3.0/2.0);
			H_QuatNorm._data[4][4] = 3*k_QuatNorm*math.pow(X._data[i*7+4][0], 2)*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 5.0/2.0) + k_QuatNorm*math.pow(X._data[i*7+4][0], 2)/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3) - k_QuatNorm*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3.0/2.0);
			H_QuatNorm._data[5][5] = 3*k_QuatNorm*math.pow(X._data[i*7+5][0], 2)*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 5.0/2.0) + k_QuatNorm*math.pow(X._data[i*7+5][0], 2)/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3) - k_QuatNorm*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3.0/2.0);
			H_QuatNorm._data[6][6] = 3*k_QuatNorm*math.pow(X._data[i*7+6][0], 2)*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 5.0/2.0) + k_QuatNorm*math.pow(X._data[i*7+6][0], 2)/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3) - k_QuatNorm*(-1 + math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), -1.0/2.0))/math.pow(math.pow(X._data[i*7+3][0], 2) + math.pow(X._data[i*7+4][0], 2) + math.pow(X._data[i*7+5][0], 2) + math.pow(X._data[i*7+6][0], 2), 3.0/2.0);
			H = math.subset(H, math.index(math.range(i*7,i*7+7), math.range(i*7,i*7+7)), math.add(math.subset(H, math.index(math.range(i*7,i*7+7), math.range(i*7,i*7+7))), H_QuatNorm));
		}
		return H;
	}
	
	function getHIquatnorm(){
		let H = math.zeros((7 * n) - 4, (7 * n) - 4); // for each point and each interval in 2D
		for(let i = 0; i < n - 1; i++){ //acts on the points and the intervals
			let H_QuatNorm = math.zeros(7,7); //temporal buffer in which the intermediate results are stored
			H_QuatNorm._data[3][4] = f_qI[i] * 3 * X._data[i*7+3][0]*X._data[i*7+4][0] / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),5);
			H_QuatNorm._data[3][5] = f_qI[i] * 3 * X._data[i*7+3][0]*X._data[i*7+5][0] / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),5);
			H_QuatNorm._data[3][6] = f_qI[i] * 3 * X._data[i*7+3][0]*X._data[i*7+6][0] / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),5);
			H_QuatNorm._data[4][5] = f_qI[i] * 3 * X._data[i*7+4][0]*X._data[i*7+5][0] / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),5);
			H_QuatNorm._data[4][6] = f_qI[i] * 3 * X._data[i*7+4][0]*X._data[i*7+6][0] / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),5);
			H_QuatNorm._data[5][6] = f_qI[i] * 3 * X._data[i*7+5][0]*X._data[i*7+6][0] / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),5);
			H_QuatNorm = math.add(H_QuatNorm,math.transpose(H_QuatNorm));
			H_QuatNorm._data[3][3] = f_qI[i] * ((3 * X._data[i*7+3][0]*X._data[i*7+3][0] / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),5)) - (1 / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),3)));
			H_QuatNorm._data[4][4] = f_qI[i] * ((3 * X._data[i*7+4][0]*X._data[i*7+4][0] / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),5)) - (1 / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),3)));
			H_QuatNorm._data[5][5] = f_qI[i] * ((3 * X._data[i*7+5][0]*X._data[i*7+5][0] / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),5)) - (1 / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),3)));
			H_QuatNorm._data[6][6] = f_qI[i] * ((3 * X._data[i*7+6][0]*X._data[i*7+6][0] / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),5)) - (1 / math.pow(math.norm(math.squeeze(math.subset(X, math.index(math.range(i*7+3,i*7+7),0)))),3)));
			H = math.subset(H, math.index(math.range(i*7,i*7+7), math.range(i*7,i*7+7)), math.add(math.subset(H, math.index(math.range(i*7,i*7+7), math.range(i*7,i*7+7))), H_QuatNorm));
		}
		return H;
	}
	
	function getHbendingtorsion(){
		let H = math.zeros((7 * n) - 4, (7 * n) - 4); // for each point and each interval in 2D
		for(let i = 0; i < n-2; i++){ //acts on each interval
			let B = math.matrix([[ X._data[i*7+11][0],-X._data[i*7+10][0],-X._data[i*7+13][0], X._data[i*7+12][0]]
								,[ X._data[i*7+12][0], X._data[i*7+13][0],-X._data[i*7+10][0],-X._data[i*7+11][0]]
								,[ X._data[i*7+13][0],-X._data[i*7+12][0], X._data[i*7+11][0],-X._data[i*7+10][0]]]);
								
			let C = math.matrix([[-X._data[i*7+ 4][0], X._data[i*7+ 3][0], X._data[i*7+ 6][0],-X._data[i*7+ 5][0]]
								,[-X._data[i*7+ 5][0],-X._data[i*7+ 6][0], X._data[i*7+ 3][0], X._data[i*7+ 4][0]]
								,[-X._data[i*7+ 6][0], X._data[i*7+ 5][0],-X._data[i*7+ 4][0], X._data[i*7+ 3][0]]]);

			let H_BendingTorsion = math.zeros(14,14);
			H_BendingTorsion = math.subset(H_BendingTorsion, math.index(math.range( 3, 7), math.range( 3, 7)), math.multiply(4/math.pow(L_ref,2), math.transpose(B),math.multiply(A, B)));
			H_BendingTorsion = math.subset(H_BendingTorsion, math.index(math.range(10,14), math.range(10,14)), math.multiply(4/math.pow(L_ref,2), math.transpose(C),math.multiply(A, C)));
			H_BendingTorsion = math.subset(H_BendingTorsion, math.index(math.range( 3, 7), math.range(10,14)), math.multiply(4/math.pow(L_ref,2), math.transpose(B),math.multiply(A, C)));
			H_BendingTorsion = math.subset(H_BendingTorsion, math.index(math.range(10,14), math.range( 3, 7)), math.multiply(4/math.pow(L_ref,2), math.transpose(C),math.multiply(A, B)));
			H = math.subset(H, math.index(math.range(i*7,i*7+14), math.range(i*7,i*7+14)), math.add(math.subset(H, math.index(math.range(i*7,i*7+14), math.range(i*7,i*7+14))), H_BendingTorsion));
		}
		return H;
	}
	
	function getHhandles(){
		let H = math.zeros((7 * n) - 4, (7 * n) - 4); // for each point and each interval in 2D
		for(let j = 0; j < handles.length; j++){
			let i = math.floor(handles[j]._data[0][0] * (n-1));
			let r = (((L_ref * (n-1)) * handles[j]._data[0]) - (i * L_ref))/L_ref;
			let H_Handles = math.zeros(10,10);
			H_Handles = math.subset(H_Handles, math.index(math.range( 0, 3), math.range( 0, 3)),math.multiply(k_Handles * math.pow(r-1,2), math.identity(3)));
			H_Handles = math.subset(H_Handles, math.index(math.range( 3, 7), math.range( 3, 7)),math.multiply(k_Handles, math.identity(4)));
			H_Handles = math.subset(H_Handles, math.index(math.range( 0, 3), math.range( 7,10)),math.multiply(-k_Handles * (r-1) * r, math.identity(3)));
			H_Handles = math.subset(H_Handles, math.index(math.range( 7,10), math.range( 0, 3)),math.multiply(-k_Handles * (r-1) * r, math.identity(3)));				
			H_Handles = math.subset(H_Handles, math.index(math.range( 7,10), math.range( 7,10)),math.multiply(k_Handles * math.pow(r,2), math.identity(3)));
			H = math.subset(H, math.index(math.range(i*7,i*7 + 10), math.range(i*7,i*7 + 10)), math.add(math.subset(H, math.index(math.range(i*7,i*7 + 10), math.range(i*7,i*7 + 10))), H_Handles));
		}
		return H;
	}
	
	//reduce matrix to only position values
	function filterPositions(data){
		if((data._size[0] == (n * 2 - 1)) && (data._size[1] == 1)){ //in case of a matrix filled with energy values
			let out = math.zeros(n,1);
			for(let i = 0; i < n; i++){
				out._data[i][0] = data._data[i*2][0];
			}
			return out;
		}else{
		}
	}
	
	function eliminateCoherence(){
		for(let i = 0; i < n - 1; i++){
			X = math.subset(X, math.index(math.range(i*7+3,i*7+7),0),quaternionDirection(math.squeeze(u[i])))
		}
	}
	
	this.debugLength = function(){
		let F = 0.0;
		for(let i = 0; i < n - 1; i++){
			F = F + L[i];
		}
		console.log(F);
	}
	this.debugCoherence = function(){
		eliminateCoherence();
	}
}

quaternionDirection = function(direction){
	ref = math.matrix([0,1,0]);
	
	q = math.zeros(4,1);
	direction = math.divide(direction,math.norm(direction));
	
	q = math.subset(q,math.index(math.range(1,4),0),math.cross(ref,direction));
	q._data[0][0] = math.sqrt(math.pow(math.norm(ref),2) * math.pow(math.norm(direction), 2)) + math.dot(ref, direction);
	q = math.divide(q, math.norm(math.squeeze(q)));
	return q;
}