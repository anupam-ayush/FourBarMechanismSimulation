        const canvas = document.getElementById("simulationCanvas");
        const ctx = canvas.getContext("2d");// for drawing


        let isSimulating = false;
        let theta2 = 0; // Current crank angle in degrees
        let prevTheta3rad = 0; // Store previous theta3 (radians) for continuity
        let prevTheta4rad = 0; // Store previous theta4 (radians) for solver initial guess
        let animationId;
        let scaleFactor;// Pixels per unit length
        let Run = true;
    

        let zoom = 1;  
        const ZOOM_STEP = 1.1; 
        let panX = 0;
        let panY = 0;
        let isPanning = false;
        let startPan = { x: 0, y: 0 };
        const MIN_ZOOM = 0.5;
        const MAX_ZOOM = 2;

        let pathPointsB = [];
        let pathPointsC = [];




        const SINGULARITY_THRESHOLD = 1e-9; 
        const NEAR_ANGLE_THRESHOLD = 0.5; 
        let currentSimRange = [0, 360];
        let rotationDirection = 1; 
        canvas.addEventListener('wheel', function(event) {
            event.preventDefault();
        
            if (event.deltaY < 0) {
                // Zoom in
                zoom *= ZOOM_STEP;
            } else {
                // Zoom out
                zoom /= ZOOM_STEP;
            }
            zoom = Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, zoom));
            applyTransform();
            drawInitialState(); // Or call redraw logic depending on simulation state
        });
        
        canvas.addEventListener('mousedown', function(event) {
            isPanning = true;
            startPan = { x: event.offsetX, y: event.offsetY };
        });
        
        canvas.addEventListener('mousemove', function(event) {
            if (isPanning) {
                const dx = event.offsetX - startPan.x;
                const dy = event.offsetY - startPan.y;
        
                panX += dx;
                panY += dy;
        
                startPan = { x: event.offsetX, y: event.offsetY };
        
                applyTransform();
                if (!isSimulating) {
                    drawInitialState();
                } else {
                    // optionally: redraw last frame of drawLinkage()
                }
            }
        });
        
        canvas.addEventListener('mouseup', () => {
            isPanning = false;
        });
        canvas.addEventListener('mouseleave', () => {
            isPanning = false;
        });
        

        function applyTransform() {
            ctx.setTransform(1, 0, 0, 1, 0, 0); // Reset transform
            ctx.translate(canvas.width / 2 + panX, canvas.height / 2 + panY); // Pan & center
            ctx.scale(zoom, -zoom); // Zoom and flip Y-axis
        }
        
        document.getElementById("resetViewButton").addEventListener("click", () => {
            zoom = 1;
            panX = 0;
            panY = 0;
            applyTransform();


            if (!isSimulating) {
                drawInitialState();
            }
        });
        
        
        // --- Canvas Setup ---
        function initCanvas() 
        {
            // Make canvas fixed size
            const canvasWidth = 800; // Match CSS
            const canvasHeight = 700; // Match CSS
            canvas.width = canvasWidth;
            canvas.height = canvasHeight;
            // Set origin to center and flip Y-axis

            // ctx.setTransform(1, 0, 0, -1, canvas.width / 2, canvas.height / 2);

            applyTransform();



            // Redraw static elements if needed when resizing
            if (!isSimulating) 
            { // Avoid redrawing initial state during resize if simulating
                drawInitialState();
            }
        }

        function drawInitialState() 
        {
            // Draw a default view or the last simulated state if available
            // For simplicity, just clear canvas
            ctx.clearRect(-canvas.width / 2, -canvas.height / 2, canvas.width, canvas.height);
            
            // draws axes
            ctx.beginPath();
            ctx.moveTo(-canvas.width/2, 0);
            ctx.lineTo(canvas.width/2, 0);
            ctx.moveTo(0, -canvas.height/2);
            ctx.lineTo(0, canvas.height/2);
            ctx.strokeStyle = "#eee";
            ctx.lineWidth = 1;
            ctx.stroke();
        }


        // --- Drawing Functions ---
        function drawLinkage(l1, l2, l3, l4, theta2_deg, theta3_deg, theta4_deg) 
        {
            if (isNaN(theta2_deg) || isNaN(theta3_deg) || isNaN(theta4_deg)) 
            {
                console.error("Draw skip: NaN Angle detected.");
                document.getElementById("statusMessage").textContent = "Error: Invalid angle calculation.";
                // Consider stopping simulation here?
                // toggleSimulation();
                return;
            }

            // Clear canvas
            ctx.clearRect(-canvas.width / 2, -canvas.height / 2, canvas.width, canvas.height);


            // Convert degrees to radians for Math functions
            const theta2Rad = math.unit(theta2_deg, "deg").toNumber("rad");
            const theta3Rad = math.unit(theta3_deg, "deg").toNumber("rad");
            const theta4Rad = math.unit(theta4_deg, "deg").toNumber("rad");

            // Calculate joint coordinates (scaled)
            const jointA = { x: 0, y: 0 }; // Fixed pivot 1 (origin)
            const jointD = { x: l1 * scaleFactor, y: 0 }; // Fixed pivot 2
            const jointB = { x: l2 * scaleFactor * Math.cos(theta2Rad), y: l2 * scaleFactor * Math.sin(theta2Rad) }; // Moving pivot on crank
            const jointC = { x: jointD.x + l4 * scaleFactor * Math.cos(theta4Rad), y: jointD.y + l4 * scaleFactor * Math.sin(theta4Rad) }; // Moving pivot on rocker
            // Alternative calculation for C based on B (good for checking consistency)
            // const jointC_alt = { x: jointB.x + l3 * scaleFactor * Math.cos(theta3Rad), y: jointB.y + l3 * scaleFactor * Math.sin(theta3Rad) };
            // console.log("C Pos Check:", jointC, jointC_alt);


            pathPointsB.push({ x: jointB.x, y: jointB.y });
            pathPointsC.push({ x: jointC.x, y: jointC.y });


            // Draw Trace Paths
            ctx.beginPath();
            for (let pt of pathPointsB) {
                ctx.moveTo(pt.x, pt.y);
                ctx.arc(pt.x, pt.y, 2 / zoom, 0, 2 * Math.PI);
            }
            ctx.fillStyle = "rgba(255,0,0,0.6)"; // Red = Point B
            ctx.fill();
            
            ctx.beginPath();
            for (let pt of pathPointsC) {
                ctx.moveTo(pt.x, pt.y);
                ctx.arc(pt.x, pt.y, 2 / zoom, 0, 2 * Math.PI);
            }
            ctx.fillStyle = "rgba(0,255,0,0.6)"; // Blue = Point C
            ctx.fill();
            ctx.beginPath();
            ctx.moveTo(-canvas.width/2, 0);
            ctx.lineTo(canvas.width/2, 0);
            ctx.moveTo(0, -canvas.height/2);
            ctx.lineTo(0, canvas.height/2);
            ctx.strokeStyle = "#eee";
            ctx.lineWidth = 1;
            ctx.stroke();




            // Draw ground link (l1)
            ctx.beginPath();
            ctx.moveTo(jointA.x, jointA.y);
            ctx.lineTo(jointD.x, jointD.y);
            ctx.strokeStyle = "#333"; // Darker for ground
            ctx.lineWidth = 4;
            ctx.stroke();

            // Draw moving links (l2, l3, l4)
            ctx.beginPath();
            ctx.moveTo(jointA.x, jointA.y); // Start at A
            ctx.lineTo(jointB.x, jointB.y); // Link l2
            ctx.lineTo(jointC.x, jointC.y); // Link l3
            ctx.lineTo(jointD.x, jointD.y); // Link l4
            ctx.strokeStyle = "#2196F3"; // Blue for moving links
            ctx.lineWidth = 3;
            ctx.stroke();

            // Draw joints
            const jointRadius = 8 ;
            const drawJoint = (point, name, color = "#E91E63") => {
                ctx.save(); // Save current transform state
                ctx.scale(1, -1); // Flip Y axis temporarily for text drawing
                ctx.beginPath();
                ctx.arc(point.x, -point.y, jointRadius, 0, Math.PI * 2);
                ctx.fillStyle = color;
                ctx.fill();

                // Draw label slightly offset
                ctx.fillStyle = "#333";
                ctx.font = "10px Arial";
                ctx.fillText(name, point.x + jointRadius + 2, -point.y - jointRadius + 2);
                ctx.restore(); // Restore original transform state
            };

            drawJoint(jointA, "A", "#555"); // Grey for fixed joints
            drawJoint(jointD, "D", "#555");
            drawJoint(jointB, "B"); // Pink for moving joints
            drawJoint(jointC, "C");

            // Draw link labels 
            ctx.font = "12px Arial";
            ctx.fillStyle = "#666";

            const drawLinkLabel = (p1, p2, name) => {
                const midX = (p1.x + p2.x) / 2;
                const midY = (p1.y + p2.y) / 2;

                ctx.save();
                ctx.scale(1, -1); // Flip for text

                // Add a slight perpendicular offset for clarity
                let dx = p2.x - p1.x;
                let dy = p2.y - p1.y;
                let len = Math.sqrt(dx*dx + dy*dy);
                let offsetX = -dy / len * 8; // Perpendicular offset
                let offsetY = dx / len * 8;

                ctx.fillText(name, midX + offsetX, -midY - offsetY);
                ctx.restore();
            };
             
            drawLinkLabel(jointA, jointB, "l2");
            drawLinkLabel(jointB, jointC, "l3");
            drawLinkLabel(jointC, jointD, "l4");
            drawLinkLabel(jointA, jointD, "l1");
        }



        // --- Calculation Functions ---

        function normalizeAngle(angle_deg) 
        {
            let normalized = angle_deg % 360;
            if (normalized < 0) 
            {
                normalized += 360;
            }
            return normalized;
        }


        function pitopi (angle) 
        {
            let normalized = angle % 360;  
            if (normalized < 0) 
            {
                normalized += 360;  
            }
            return normalized > 180 ? normalized - 360 : normalized;
        }

      
        function Bisection(theta4_guess_rad, func) 
        {
            let a = 0;
            let b = 2 * Math.PI;
            let fa = func(a);
            let fb = func(b);
            if (fa * fb >= 0) {
                console.warn("Bisection: Initial bounds [0, 2pi] do not bracket a root. Trying guess +/- range.");
                let delta = Math.PI / 4;
                a = Math.max(0, theta4_guess_rad - delta);
                b = Math.min(2*Math.PI, theta4_guess_rad + delta);
                fa = func(a);
                fb = func(b);
                if (fa * fb >= 0) {
                    console.error("Bisection failed: Could not find a suitable bracket.");
                  return NaN; 
                }
            }
            let mid;
            let tolerance = 1e-9; 
            let maxIter = 100;
            for (let i = 0; i < maxIter; i++) {
                mid = (a + b) / 2;
                let fmid = func(mid);
                if (Math.abs(fmid) < tolerance || (b - a) / 2 < tolerance) {
                    return mid; 
                }
                if (fa * fmid < 0) {
                    b = mid;
                    fb = fmid; // Update fb
                } else {
                    a = mid;
                    fa = fmid; // Update fa
                }
            }

            console.warn("Bisection reached max iterations without converging to tolerance.");
            return mid;         }

        // Numerical solver (Newton-Raphson - faster convergence)
        function newtonraphson(theta2rad, k1, k2, k3, theta4_prev_rad) 
        {
              let current_theta4_rad = theta4_prev_rad; 
                if (Math.abs(theta4_prev_rad) < 1e-6) {
                let cos_t4_minus_t2_approx = k1 - k2 * Math.cos(theta2rad) + k3;
                if (Math.abs(cos_t4_minus_t2_approx) <= 1) {
                    let t4_minus_t2_approx = Math.acos(cos_t4_minus_t2_approx);
                    current_theta4_rad =( t4_minus_t2_approx + theta2rad);
                    if (isNaN(current_theta4_rad)) current_theta4_rad = theta2rad; 
                } else {
                    current_theta4_rad = theta2rad;
                }
            }
            const func = (th4) => k1 * Math.cos(th4) - k2 * Math.cos(theta2rad) + k3 - Math.cos(th4 - theta2rad);
            const dfunc = (th4) => -k1 * Math.sin(th4) + Math.sin(th4 - theta2rad);
            let maxIter = 100;
            let tolerance = 1e-9; 
            let iter = 0;
            while (iter < maxIter) 
            {
                let fx = func(current_theta4_rad);
                if (Math.abs(fx) <= tolerance) {
                    return current_theta4_rad; // Converged
                }
                let dfx = dfunc(current_theta4_rad);
                if (Math.abs(dfx) < SINGULARITY_THRESHOLD) {
                    console.warn(`NR Warning: Derivative near zero at theta2=${(theta2rad*180/Math.PI).toFixed(1)} deg. Trying Bisection.`);
                    // Fallback to Bisection, using current estimate as guess
                    return Bisection(current_theta4_rad, func);
                }
                current_theta4_rad = current_theta4_rad - fx / dfx;
                iter++;
            }
            console.warn(`NR Warning: Max iterations (${maxIter}) reached at theta2=${(theta2rad*180/Math.PI).toFixed(1)} deg. Trying Bisection.`);
           return Bisection(current_theta4_rad, func);
        }

        // --- POSITION ANALYSIS ---
        function positionAnalysis(theta2_deg, l1, l2, l3, l4, prevTheta4rad) {

            if (l1 <= 0 || l2 <= 0 || l3 <= 0 || l4 <= 0) {
                console.error("Position Analysis Error: Link lengths must be positive.");
                document.getElementById("statusMessage").textContent = "Error: Link lengths must be positive.";
                return null; 
            }
            const links = [l1, l2, l3, l4];
            const maxL = Math.max(...links);
            const sumOthers = links.reduce((sum, l) => sum + l, 0) - maxL;
            if (maxL > sumOthers + SINGULARITY_THRESHOLD) {
                console.error("Position Analysis Error: Linkage cannot be assembled (max link > sum of others).");
                document.getElementById("statusMessage").textContent = "Error: Linkage cannot assemble.";
                return null;
            }


            let theta2rad = math.unit(theta2_deg, "deg").toNumber("rad");

            let isParallelogram = Math.abs(l1 - l3) < SINGULARITY_THRESHOLD && Math.abs(l2 - l4) < SINGULARITY_THRESHOLD;
            let theta2NormDeg = normalizeAngle(theta2_deg);
           
            if (isParallelogram) {
               if (Math.abs(theta2NormDeg) < NEAR_ANGLE_THRESHOLD || Math.abs(theta2NormDeg - 360) < NEAR_ANGLE_THRESHOLD) {
                    console.log(`Pos Analysis: Parallelogram near 0 deg. Forcing theta3=0, theta4=0.`);
                    return ["0.00", "0.00", 0]; // Return [theta3_deg, theta4_deg, theta4_rad]
                }
             else if (theta2NormDeg > 180 ) {
                    console.log(`Pos Analysis: Parallelogram near 180 deg. Forcing theta3=180, theta4=180.`);
                    return ["0.00", `${theta2_deg}`, Math.PI]; // Return [theta3_deg, theta4_deg, theta4_rad]
                }
           }

            // --- Standard Calculation using Freudenstein's constants ---
            let k1 = l1 / l2;
            let k2 = l1 / l4;
            let k3 = (l1 ** 2 + l2 ** 2 - l3 ** 2 + l4 ** 2) / (2 * l2 * l4);

            // Solve for theta4 using Newton-Raphson (with Bisection failsafe)
            let currentTheta4rad = newtonraphson(theta2rad, k1, k2, k3, prevTheta4rad);

            if (currentTheta4rad === null || isNaN(currentTheta4rad)) {
                console.error(`Position Analysis Error: Solver failed to find theta4 for theta2 = ${theta2_deg.toFixed(2)} deg.`);
                document.getElementById("statusMessage").textContent = "Error: Position solver failed.";
                return null;
            }

            let X_comp = l1 + l4 * Math.cos(currentTheta4rad) - l2 * Math.cos(theta2rad);
            let Y_comp = l4 * Math.sin(currentTheta4rad) - l2 * Math.sin(theta2rad);
            let currentTheta3rad = Math.atan2(Y_comp, X_comp);
            let theta3_deg_norm = normalizeAngle(math.unit(currentTheta3rad, "rad").toNumber("deg"));
            let theta4_deg_norm = normalizeAngle(math.unit(currentTheta4rad, "rad").toNumber("deg"));
            let checkX = l2*Math.cos(theta2rad) + l3*Math.cos(currentTheta3rad);
            let targetX = l1 + l4*Math.cos(currentTheta4rad);
            let checkY = l2*Math.sin(theta2rad) + l3*Math.sin(currentTheta3rad);
            let targetY = l4*Math.sin(currentTheta4rad);
            if (Math.abs(checkX - targetX) > 1e-3 || Math.abs(checkY - targetY) > 1e-3) {
                console.warn(`Consistency check failed slightly at theta2=${theta2_deg.toFixed(1)}: `+
                             `X: ${checkX.toFixed(4)} vs ${targetX.toFixed(4)}, Y: ${checkY.toFixed(4)} vs ${targetY.toFixed(4)}`);
            }
          return [theta3_deg_norm.toFixed(2), theta4_deg_norm.toFixed(2), currentTheta4rad];
        }
        function togglePoints(l1, l2, l3, l4) 
        {
            // Sort links: s=shortest, l=longest, p,q=others
            let links = [
                { len: l1, type: 'l1' },
                { len: l2, type: 'l2' },
                { len: l3, type: 'l3' },
                { len: l4, type: 'l4' }
            ];

            links.sort((a, b) => a.len - b.len);
            let s = links[0].len, sType = links[0].type;
            let l = links[3].len;
            let p = links[1].len;
            let q = links[2].len;

            let isGrashof = (s + l <= p + q + SINGULARITY_THRESHOLD); // Add tolerance
            let linkageType = "";
            let fullRotationPossible = false;

            let isPara = Math.abs(l1 - l3) < SINGULARITY_THRESHOLD && Math.abs(l2 - l4) < SINGULARITY_THRESHOLD;
            let isAntiPara = Math.abs(l1 - l2) < SINGULARITY_THRESHOLD && Math.abs(l3 - l4) < SINGULARITY_THRESHOLD; // Or other pairs

            if (isPara) 
            {
                linkageType = "Parallelogram";
                fullRotationPossible = true; 
            } else if (isAntiPara) {
                linkageType = "Antiparallelogram (Galloway)";
                isGrashof = true; 
                if (sType === 'l2' || sType === 'l1') { fullRotationPossible = true; } // If l2 is shortest or fixed is shortest
            }
            else if (isGrashof) {
                linkageType = "Grashof ";
                if (sType === 'l1') { linkageType += "Double Crank"; fullRotationPossible = true; }
                else if (sType === 'l2') { linkageType += "Crank-Rocker"; fullRotationPossible = true; }
                else if (sType === 'l3') { linkageType += "Double Rocker (R-C-R)"; fullRotationPossible = false; } // Input l2 cannot rotate 360
                else if (sType === 'l4') { linkageType += "Double Rocker (C-R-R)"; fullRotationPossible = false; } // Input l2 cannot rotate 360
                if (Math.abs(s + l - (p + q)) < SINGULARITY_THRESHOLD) {
                    linkageType += "(Change Point)";
                }
            } else {
                linkageType = "Non-Grashof Triple Rocker";
                fullRotationPossible = false;
            }
            document.getElementById("Type").textContent = `${linkageType}`;
            console.log(`Linkage Type: ${linkageType}`);

            if (fullRotationPossible) {
                console.log("Expected full rotation [0, 360] deg.");
                return [0, 360,1]; // Full rotation expected
            } else {
               const val1 = (l1 ** 2 + l2 ** 2 - (l3 + l4) ** 2) / (2 * l1 * l2);
               const val2 = (l1 ** 2 + l2 ** 2 - (l3 - l4) ** 2) / (2 * l1 * l2);
               const theta2max = (Math.acos(Math.max(-1, Math.min(1, val1))) * 180 / Math.PI);
               const theta2min =(Math.acos(Math.max(-1, Math.min(1, val2))) * 180 / Math.PI);

               if (Math.abs(val1) > 1 && Math.abs(val2) > 1) 
                {
                    return [0, 360,1];
                }
               if (Math.abs(val1) < 1 && Math.abs(val2) < 1) 
                {
                    return [theta2min, theta2max,1];
                }
               if (Math.abs(val1) < 1 && Math.abs(val2) > 1) 
                {
                   return [theta2max,360 - theta2max,-1];
                }

                return [theta2min,360-theta2min,1];
            }
        }


            


       
        function simulate()
        {
           
            const _l1 = parseFloat(document.getElementById("l1").value);
            const _l2 = parseFloat(document.getElementById("l2").value);
            const _l3 = parseFloat(document.getElementById("l3").value);
            const _l4 = parseFloat(document.getElementById("l4").value);
            const initialTheta2 = parseFloat(document.getElementById("initialTheta2").value); 
            const finalTheta2 = parseFloat(document.getElementById("finalTheta2").value);   
            const deltaTheta2 = parseFloat(document.getElementById("deltaTheta2").value); 
            const simSpeed = parseFloat(document.getElementById("simSpeed").value);
            const angularvel2 = parseFloat(document.getElementById("angularvel2").value);         
            const fixed = document.getElementById("fixedLink").value;
            let l1 = _l1, l2 = _l2, l3 = _l3, l4 = _l4;

            // Rotate links according to inversion
            if (fixed === "l2") {
                [l1, l2, l3, l4] = [_l2, _l3, _l4, _l1];
            } else if (fixed === "l3") {
                [l1, l2, l3, l4] = [_l3, _l4, _l1, _l2];
            } else if (fixed === "l4") {
                [l1, l2, l3, l4] = [_l4, _l1, _l2, _l3];
            }
            
            
            // --- Validate Inputs (Keep these checks) ---
            if (isNaN(l1) || isNaN(l2) || isNaN(l3) || isNaN(l4) || l1<=0 || l2<=0 || l3<=0 || l4<=0) {
                document.getElementById("statusMessage").textContent = "Error: Invalid link lengths.";
                toggleSimulation(); return; 
            }
            if (isNaN(deltaTheta2) || deltaTheta2 <= 0) {
                document.getElementById("statusMessage").textContent = "Error: Invalid angle step size.";
                toggleSimulation(); return; 
            }
            if (isNaN(simSpeed) || simSpeed <= 0) {
                document.getElementById("statusMessage").textContent = "Error: Invalid speed.";
                toggleSimulation(); return; 
                        }

            const minAngle = currentSimRange[0];
            const maxAngle = currentSimRange[1];
            const x = currentSimRange[2];
            if(x===-1 && Run ){
                rotationDirection = -1;
                Run = false;
            }
           
            // --- Update Angle Step ---
            let deltaAngle = deltaTheta2 * simSpeed * 0.05;
            let nextTheta2 = normalizeAngle(theta2 + deltaAngle*rotationDirection); 
            let analysisTheta2;
            
            if(x === 1){
                analysisTheta2 = Math.max(minAngle, Math.min(maxAngle, theta2));
            }
            else if (x === -1 && nextTheta2<360 && nextTheta2>=maxAngle-2){
                analysisTheta2 = Math.max(minAngle, Math.max(maxAngle, nextTheta2));
            }
            
            else {
                analysisTheta2 = Math.min(minAngle, Math.min(maxAngle, nextTheta2));
            }
        
            theta2 = nextTheta2;
            
           
            const positionResult = positionAnalysis(analysisTheta2, l1, l2, l3, l4, prevTheta4rad);
            
            if (!positionResult) {
               
                console.warn(`Singularity detected or solver failure near theta2=${analysisTheta2.toFixed(1)}. Reversing direction.`);
                rotationDirection *= -1;

              
                theta2 = normalizeAngle(theta2 + deltaAngle * rotationDirection * 3);

               
                requestAnimationFrame(simulate);
              
                return;
            }

            const [theta3Str, theta4Str, currentTheta4rad] = positionResult;
            const theta3 = parseFloat(theta3Str); 
            const theta4 = parseFloat(theta4Str);
            const currentTheta3rad = math.unit(theta3, "deg").toNumber("rad");
           
            prevTheta4rad = currentTheta4rad;
            prevTheta3rad = currentTheta3rad;
           
            document.getElementById("THETA2").textContent = `Theta2:θ2 (${analysisTheta2.toFixed(0)}°)`;
            document.getElementById("THETA3").textContent = `Theta3:θ3 (${pitopi(theta3.toFixed(0))}°)`;
            document.getElementById("THETA4").textContent = `Theta4:θ4 (${pitopi(theta4.toFixed(0))}°)`;
            document.getElementById("Input range").textContent = `θ2 Range:(${currentSimRange[0].toFixed(0)}°,${currentSimRange[1].toFixed(0)}°)`;

            const analysisTheta2Rad = math.unit(analysisTheta2, "deg").toNumber("rad");
            const base_w2_deg_per_sec =  angularvel2 ; 
            const w2_rad_per_sec = base_w2_deg_per_sec*rotationDirection;
            const alpha2 = 0; 

            let omega3_rad_per_sec = NaN;
            let omega4_rad_per_sec = NaN;
            let alpha3_rad_per_sec2 = NaN;
            let alpha4_rad_per_sec2 = NaN;

          
            const sin_t3_minus_t4 = Math.sin(currentTheta3rad - currentTheta4rad);
            const isSingular = Math.abs(sin_t3_minus_t4) < SINGULARITY_THRESHOLD;
            const isParallelogramNearSingular = (Math.abs(l1 - l3) < SINGULARITY_THRESHOLD && Math.abs(l2 - l4) < SINGULARITY_THRESHOLD && isSingular);


            if (isSingular && !isParallelogramNearSingular) {
                console.warn(`V&A Singularity Detected (non-parallelogram, sin(t3-t4)≈0) at t2=${analysisTheta2.toFixed(1)}`);
               
                omega4_rad_per_sec = 0;
                let cos_t3 = Math.cos(currentTheta3rad);
                if (Math.abs(cos_t3) > SINGULARITY_THRESHOLD) {
                    omega3_rad_per_sec = - (l2 * w2_rad_per_sec * Math.cos(analysisTheta2Rad)) / (l3 * cos_t3);
                } else {
                     let sin_t3 = Math.sin(currentTheta3rad); 
                     if (Math.abs(sin_t3) > SINGULARITY_THRESHOLD) {
                       omega3_rad_per_sec = - (-l2 * w2_rad_per_sec * Math.sin(analysisTheta2Rad)) / (l3 * sin_t3);
                     } else {
                         omega3_rad_per_sec = 0; 
                         console.error("Double singularity? cos(t3) and sin(t3) near zero.");
                     }
                }

                
                alpha3_rad_per_sec2 = 0;
                alpha4_rad_per_sec2 = 0;
                console.log(`--> Setting w4=0, w3=${omega3_rad_per_sec.toFixed(3)}, a3=0, a4=0`);

            } else if (isParallelogramNearSingular) {
                 console.log(`V&A Parallelogram Singularity Detected at t2=${analysisTheta2.toFixed(1)}`);
                 omega3_rad_per_sec = 0;
                 omega4_rad_per_sec = w2_rad_per_sec; // w4 = w2
                 alpha3_rad_per_sec2 = 0;
                 alpha4_rad_per_sec2 = alpha2; // a4 = a2 (which is 0 here)
                 console.log(`--> Setting w3=0, w4=w2=${w2_rad_per_sec.toFixed(3)}, a3=0, a4=a2=${alpha2}`);

            } else {
                let A_vel = math.matrix([
                    [-l3 * Math.sin(currentTheta3rad), l4 * Math.sin(currentTheta4rad)],
                    [ l3 * Math.cos(currentTheta3rad), -l4 * Math.cos(currentTheta4rad)]
                ]);
                let B_vel = math.matrix([
                    [l2 * w2_rad_per_sec * Math.sin(analysisTheta2Rad)],
                    [-l2 * w2_rad_per_sec * Math.cos(analysisTheta2Rad)]
                ]);
                try {
                    let W = math.multiply(math.inv(A_vel), B_vel);
                    omega3_rad_per_sec = W.get([0, 0]);
                    omega4_rad_per_sec = W.get([1, 0]); 
                } catch (e) {
                    console.error("Velocity matrix solution failed unexpectedly:", e);
                    omega3_rad_per_sec = NaN;
                    omega4_rad_per_sec = NaN;
                }

                 if (!isNaN(omega3_rad_per_sec) && !isNaN(omega4_rad_per_sec)) {

                    let A_accel = math.matrix([
                        [-l3 * Math.sin(currentTheta3rad),  l4 * Math.sin(currentTheta4rad)],
                        [ l3 * Math.cos(currentTheta3rad), -l4 * Math.cos(currentTheta4rad)]
                    ]);
                    let B1 = l2*alpha2*Math.sin(analysisTheta2Rad) + l2*w2_rad_per_sec**2*Math.cos(analysisTheta2Rad) + l3*omega3_rad_per_sec**2*Math.cos(currentTheta3rad) - l4*omega4_rad_per_sec**2*Math.cos(currentTheta4rad);
                    let B2 = -l2*alpha2*Math.cos(analysisTheta2Rad) + l2*w2_rad_per_sec**2*Math.sin(analysisTheta2Rad) + l3*omega3_rad_per_sec**2*Math.sin(currentTheta3rad) - l4*omega4_rad_per_sec**2*Math.sin(currentTheta4rad);
                    let B_accel = math.matrix([[B1], [B2]]);

                    try {
                        let Alpha_vec = math.multiply(math.inv(A_accel), B_accel);
                        alpha3_rad_per_sec2 = Alpha_vec.get([0, 0]);
                        alpha4_rad_per_sec2 = Alpha_vec.get([1, 0]);
                    } catch (error) {
                         console.error("Acceleration matrix solution failed unexpectedly:", error);
                         alpha3_rad_per_sec2 = NaN;
                         alpha4_rad_per_sec2 = NaN;
                    }
                 } else {
                    alpha3_rad_per_sec2 = NaN;
                    alpha4_rad_per_sec2 = NaN;
                 }
            }


            
            const omega4_to_use = omega4_rad_per_sec;
            const alpha4_to_use = alpha4_rad_per_sec2;

            const vBx = -w2_rad_per_sec * l2 * Math.sin(analysisTheta2Rad);
            const vBy = w2_rad_per_sec * l2 * Math.cos(analysisTheta2Rad);
            let vCx = NaN, vCy = NaN;
            if (!isNaN(omega4_to_use)) {
                vCx = -omega4_to_use * l4 * Math.sin(currentTheta4rad);
                vCy = omega4_to_use * l4 * Math.cos(currentTheta4rad);
            }
            // --- Update Velocity Display ---
            document.getElementById("jointBVel").textContent = `B: v=(${vBx.toFixed(2)}, ${vBy.toFixed(2)})`;
            document.getElementById("jointCVel").textContent = `C: v=(${vCx.toFixed(2)}, ${vCy.toFixed(2)})`;
            document.getElementById("omega3Disp").textContent = `ω3 = (${isNaN(omega3_rad_per_sec) ? "--" : omega3_rad_per_sec.toFixed(2)}) rad/s`;
            document.getElementById("omega4Disp").textContent = `ω4 = (${isNaN(omega4_to_use) ? "--" : omega4_to_use.toFixed(2)}) rad/s`;

            // --- Drawing (uses angles corresponding to analysisTheta2) ---
            drawLinkage(l1, l2, l3, l4, analysisTheta2, theta3, theta4);

            // --- Linear Acceleration Calculations (based on analysisTheta2Rad) ---
            const aBx = -alpha2 * l2 * Math.sin(analysisTheta2Rad) - w2_rad_per_sec ** 2 * l2 * Math.cos(analysisTheta2Rad);
            const aBy = alpha2 * l2 * Math.cos(analysisTheta2Rad) - w2_rad_per_sec ** 2 * l2 * Math.sin(analysisTheta2Rad);
            let aCx = NaN, aCy = NaN;
             if (!isNaN(alpha4_to_use) && !isNaN(omega4_to_use)) { // Check omega4 too
                aCx = -alpha4_to_use * l4 * Math.sin(currentTheta4rad) - omega4_to_use ** 2 * l4 * Math.cos(currentTheta4rad);
                aCy = alpha4_to_use * l4 * Math.cos(currentTheta4rad) - omega4_to_use ** 2 * l4 * Math.sin(currentTheta4rad);
            }

            // --- Update Acceleration Display ---
            document.getElementById("alpha3").textContent = `α3 = (${isNaN(alpha3_rad_per_sec2) ? "--" : alpha3_rad_per_sec2.toFixed(2)}) rad/s²`;
            document.getElementById("alpha4").textContent = `α4 = (${isNaN(alpha4_to_use) ? "--" : alpha4_to_use.toFixed(2)}) rad/s²`;
            document.getElementById("jointBAcc").textContent = `B: a=(${aBx.toFixed(2)}, ${aBy.toFixed(2)})`;
            document.getElementById("jointCAcc").textContent = `C: a=(${aCx.toFixed(2)}, ${aCy.toFixed(2)})`;


            // --- Determine Next Step Direction/Reset (using the unclamped theta2) ---
            // ... (Keep existing range checking and direction reversal logic) ...
            const isLimitedRange = (maxAngle - minAngle) < (360 - NEAR_ANGLE_THRESHOLD);

            if (isLimitedRange && x === 1) {
                // Check if the *intended* angle hit or exceeded the boundary
                if (theta2 >= maxAngle && rotationDirection === 1) {
                    theta2 = maxAngle; // Clamp the *state* angle for the next frame start
                    rotationDirection = -1; // Reverse direction
                } else if (theta2 <= minAngle && rotationDirection === -1) {
                    theta2 = minAngle; // Clamp the *state* angle
                    rotationDirection = 1; // Reverse direction
                }
                // Ensure theta2 state doesn't drift due to overshoot before clamping
                theta2 = Math.max(minAngle, Math.min(maxAngle, theta2));

            }
             else if (isLimitedRange && x === -1) {
                // Check if the *intended* angle hit or exceeded the boundary
                if (theta2 <= maxAngle && theta2>minAngle && rotationDirection === -1) {
                    theta2 = maxAngle; // Clamp the *state* angle for the next frame start
                    rotationDirection = 1; // Reverse direction
                                     
                } else if (theta2 >= minAngle && theta2<=maxAngle && rotationDirection === 1) {
                    theta2 = minAngle; // Clamp the *state* angle
                    rotationDirection = -1; // Reverse direction
                         
                }
                // Ensure theta2 state doesn't drift due to overshoot before clamping
                if (theta2>=maxAngle && theta2<360){
                theta2 = Math.max(minAngle, Math.max(maxAngle, theta2));}
                else if (theta2>=0 && theta2<=minAngle){
                    theta2 = Math.min(minAngle, Math.min(maxAngle, theta2));
                }

            }
            else {
                // Full Rotation or User-Defined Range (logic remains the same using theta2)
                let resetOccurred = false;
                if (finalTheta2 !== initialTheta2 && Math.abs(finalTheta2 - initialTheta2) % 360 !== 0) {
                   if (rotationDirection === 1 && theta2 >= finalTheta2) { theta2 = initialTheta2; resetOccurred = true; }
                   else if (rotationDirection === -1 && theta2 <= finalTheta2) { theta2 = initialTheta2; resetOccurred = true; } // Basic reverse check

                   if (resetOccurred) { prevTheta3rad = 0; prevTheta4rad = 0; rotationDirection = 1; }
                }
                if (!resetOccurred) {
                   if (theta2 >= 360) { theta2 = theta2 % 360; }
                   else if (theta2 < 0) { theta2 = (theta2 % 360) + 360; }
                }
            }
            // --- Request Next Frame ---
            if (isSimulating) {
                animationId = requestAnimationFrame(simulate);
            } else { console.log("Simulation stopped."); }
        } // End of simulate function



        // --- Simulation Control ---
        function toggleSimulation() 
        {
            isSimulating = !isSimulating;
            const button = document.getElementById("toggleSimulation");
            button.textContent = isSimulating ? "Stop Simulation" : "Start Simulation";
            button.style.backgroundColor = isSimulating ? "#d9534f" : "#007bff"; // Red when running, blue when stopped

            document.getElementById("statusMessage").textContent = ""; // Clear previous status

            if (isSimulating) 
            {
                // Get initial values and calculate range ONCE
                const _l1 = parseFloat(document.getElementById("l1").value);
                const _l2 = parseFloat(document.getElementById("l2").value);
                const _l3 = parseFloat(document.getElementById("l3").value);
                const _l4 = parseFloat(document.getElementById("l4").value);
                theta2 = parseFloat(document.getElementById("initialTheta2").value);

///////////////////////////////inversion
                const fixed = document.getElementById("fixedLink").value;
                let l1 = _l1, l2 = _l2, l3 = _l3, l4 = _l4;

                // Rotate links according to inversion
                if (fixed === "l2") {
                    [l1, l2, l3, l4] = [_l2, _l3, _l4, _l1];
                } else if (fixed === "l3") {
                    [l1, l2, l3, l4] = [_l3, _l4, _l1, _l2];
                } else if (fixed === "l4") {
                    [l1, l2, l3, l4] = [_l4, _l1, _l2, _l3];
                }


                // --- *** IMPROVED DYNAMIC SCALING CALCULATION *** ---
                const canvasWidth = canvas.width;
                const canvasHeight = canvas.height;
                            
                // Use a safe margin to avoid clipping
                const margin = 0.1; // 10% margin on each side
                const effectiveWidth = canvasWidth * (1 - 2 * margin);
                const effectiveHeight = canvasHeight * (1 - 2 * margin);
                            
                // Calculate the absolute maximum reach of the linkage
                const maxReach = l1 + l2 + l3 + l4;
                            
                // Determine the maximum dimension to scale against (square canvas approach)
                const maxDimension = Math.max(effectiveWidth, effectiveHeight);
                            
                // Calculate the scale factor to fit the maxReach within the effective dimension
                scaleFactor = maxDimension / maxReach;
                            
                // Ensure the scale factor is positive and non-zero
                if (!scaleFactor || scaleFactor <= 0) {
                    console.warn("Invalid scale factor calculated, defaulting to a safe value.");
                    scaleFactor = Math.min(canvasWidth, canvasHeight) / (Math.max(l1,l2,l3,l4)*4); // Safer fallback
                }
                
                console.log("Calculated Scale Factor:", scaleFactor);
                // --- *** END OF IMPROVED DYNAMIC SCALING CALCULATION *** ---


                // Validate lengths before calculating range
                if (isNaN(l1) || isNaN(l2) || isNaN(l3) || isNaN(l4) || l1<=0 || l2<=0 || l3<=0 || l4<=0) {
                    document.getElementById("statusMessage").textContent = "Error: Invalid link lengths.";
                    isSimulating = false; // Prevent starting
                    button.textContent = "Start Simulation";
                    button.style.backgroundColor = "#007bff";
                    return;
                }

                // Calculate and store the simulation range
                currentSimRange = togglePoints(l1, l2, l3, l4);
                const minAngle = currentSimRange[0];
                const maxAngle = currentSimRange[1];
                const x = currentSimRange[2];


                console.log(`Calculated Range: [${minAngle.toFixed(2)}, ${maxAngle.toFixed(2)},${x}]`);

                // Check assembly possibility before starting
                const links = [l1, l2, l3, l4];
                const maxL = Math.max(...links);
                const sumOthers = links.reduce((sum, l) => sum + l, 0) - maxL;
                if (maxL > sumOthers + SINGULARITY_THRESHOLD) {
                    document.getElementById("statusMessage").textContent = "Error: Linkage cannot assemble.";
                    isSimulating = false; // Prevent starting
                    button.textContent = "Start Simulation";
                    button.style.backgroundColor = "#007bff";
                    return;
                }

                // Check if calculated range is valid (not [0,0] from error)
                if (maxAngle === 0 && minAngle === 0 && !(maxL > sumOthers + SINGULARITY_THRESHOLD)) {
                   // This might happen if togglePoints failed unexpectedly but assembly is possible
                    // Check if it's because it truly cannot move (e.g., l1+l2 = l3+l4 exactly?)
                    console.warn("Motion range calculated as [0, 0]. Check geometry.");
                    // document.getElementById("statusMessage").textContent = "Warning: Linkage may be locked.";
                    // Decide whether to proceed or stop. Let's allow trying for now.
                }


                // Adjust initialTheta2 if it's outside the calculated valid range
                if ((theta2 < minAngle - NEAR_ANGLE_THRESHOLD || theta2 > maxAngle + NEAR_ANGLE_THRESHOLD) && x ===1) {
                    console.warn(`Initial angle ${theta2.toFixed(1)} is outside the valid range [${minAngle.toFixed(1)}, ${maxAngle.toFixed(1)}]. Starting at ${minAngle.toFixed(1)}.`);
                    theta2 = minAngle;
                    document.getElementById("initialTheta2").value = theta2.toFixed(1); // Update input field
                    document.getElementById("statusMessage").textContent = "Warning: Initial angle adjusted to fit linkage range.";
                }
                if ((theta2 > minAngle - NEAR_ANGLE_THRESHOLD || theta2 < maxAngle + NEAR_ANGLE_THRESHOLD) && x ===-1) {
                    console.warn(`Initial angle ${theta2.toFixed(1)} is outside the valid range [${minAngle.toFixed(1)}, ${maxAngle.toFixed(1)}]. Starting at ${minAngle.toFixed(1)}.`);
                    theta2 = minAngle;
                    document.getElementById("initialTheta2").value = theta2.toFixed(1); // Update input field
                    document.getElementById("statusMessage").textContent = "Warning: Initial angle adjusted to fit linkage range.";
                }


                // Reset state variables
                prevTheta3rad = 0; // Reset solver state
                prevTheta4rad = 0; // Reset solver state
                rotationDirection = 1; // Reset direction to forward

/////////////////////////////////////path tracing
                pathPointsB = [];
                pathPointsC = [];

                console.log("Starting simulation...");
                simulate(); // Start the animation loop
            } else {
                cancelAnimationFrame(animationId); // Stop the loop
                console.log("Simulation stopped by user.");
                Run = true;
                // Optionally clear displays when stopped
                // document.getElementById("jointBVel").textContent = `B: v=(--)`; ... etc.
            }
        }


        // --- Initial Setup & Event Listeners ---
        initCanvas(); // Draw initial canvas state
        drawInitialState(); // Draw axes or default view
        window.addEventListener("resize", initCanvas); // Adjust canvas on window resize
        document.getElementById("toggleSimulation").addEventListener("click", toggleSimulation); // Button click listener

        // Optional: Add listeners to input fields to redraw static view on change?
        const inputs = document.querySelectorAll('.input-controls input[type="number"]');
        inputs.forEach(input => {
            input.addEventListener('change', () => {
                if (!isSimulating) 
                {
                    // Try to draw the linkage at the initial angle with new lengths
                    // Need to handle potential errors if assembly fails here too
                    // ... get other lengths ...
                    const _l1 = parseFloat(document.getElementById("l1").value);
                    const _l2 = parseFloat(document.getElementById("l2").value);
                    const _l3 = parseFloat(document.getElementById("l3").value);
                    const _l4 = parseFloat(document.getElementById("l4").value);
    
                    const initTheta2 = parseFloat(document.getElementById("initialTheta2").value);
/////////////////////////////////inversion
                    const fixed = document.getElementById("fixedLink").value;
                    let l1 = _l1, l2 = _l2, l3 = _l3, l4 = _l4;
                    
                    // Rotate links according to inversion
                    if (fixed === "l2") {
                        [l1, l2, l3, l4] = [_l2, _l3, _l4, _l1];
                    } else if (fixed === "l3") {
                        [l1, l2, l3, l4] = [_l3, _l4, _l1, _l2];
                    } else if (fixed === "l4") {
                        [l1, l2, l3, l4] = [_l4, _l1, _l2, _l3];
                    }

                    const posResult = positionAnalysis(initTheta2, l1, l2, l3, l4, 0);
                    if (posResult) {
                         drawLinkage(l1, l2, l3, l4, initTheta2, parseFloat(posResult[0]), parseFloat(posResult[1]));
                    } else {
                         drawInitialState(); // Clear if position fails
                    }
                }
            });
        });

        document.getElementById("fixedLink").addEventListener('change', () => {
            pathPointsB = [];
            pathPointsC = [];
            if (!isSimulating) {
                drawInitialState();
            }
        });