classdef Camera
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.

    properties
        % Flags
        DEBUG = false;
        POSE_PLOT = false;
        DEBUG_BALL_DETECTION = false;

        % Image Processing Variables

        % Colors
        lime;
        magenta;
        orange;
        yellow;

        ourColorRGBs;
        ourColorNames;
        ourColorEnums;

        % Properties
        params;
        cam;
        camPose;
        baseImg;

        transform;

    end

    methods

        function self = Camera()
            % CAMERA Construct an instance of this class
            self.cam = webcam(); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function

            % Initialze colors
            % YONI COLORS
            % self.lime = [137 153 41];
            % self.magenta = [172 44 83];
            % self.orange = [160 33 17];
            % self.yellow = [189 153 42];

            % NATHAN COLORS
            self.lime = [125 141 32];
            self.magenta = [152 37 50];
            self.orange = [180 66 5];
            self.yellow = [140 96 0];

            self.ourColorRGBs = [self.lime; self.magenta; self.orange; self.yellow];
            self.ourColorNames = {'lime', 'magenta', 'orange', 'yellow'};
            self.ourColorEnums = {Colors.Lime, Colors.Magenta, Colors.Orange, Colors.Yellow};
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end

        function [pose, color] = detectBestBall(self)
            % DETECTBESTBALL Detect Balls + Report back positions + colors
            % Note: you do NOT need to use this function, this is just a recommendation

        end

        function params = calibrate(self)
            % CALOBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            DEBUG = self.DEBUG;
            params = 0;

            try
                % COMMENT THESE LINES TO REMOVE THE ANNOYING PAUSE
                % disp("Clear surface of any items, then press any key to continue");
                % pause;
                camcalib; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                getReport(exception);
                disp("No camerea calibration file found. Plese run camera calibration");
            end

        end

        function pose = getCameraPose(self)
            % GETCAMERAPOSE Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % There are a few debugging options included as well! Simply set POSE_PLOT
            % to true to show the checkerboard frame of reference on the picture!

            % 1. Capture image from camera
            rawImg = snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newOrigin] = undistortImage(rawImg, self.params, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img);
            % 4. Adjust imagePoints so they are in the same frame of
            % reference as original image
            % imagePoints = imagePoints + newOrigin;
            % 5. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, self.params);

            pose = [R, t';
                0, 0, 0, 1];

            if self.POSE_PLOT
                axesPoints = worldToImage(self.params, R, t, [0 0 0; 0 50 0; 50 0 0]);

                x1 = [axesPoints(1, 1), axesPoints(2, 1)]';
                y1 = [axesPoints(1, 2), axesPoints(2, 2)]';

                img = insertText(img, [x1(2), y1(2)], 'Y Axis', 'TextColor', 'green', 'FontSize', 18);
                x2 = [axesPoints(1, 1), axesPoints(3, 1)]';
                y2 = [axesPoints(1, 2), axesPoints(3, 2)]';

                img = insertText(img, axesPoints(3, 1:2), 'X Axis', 'TextColor', 'red', 'FontSize', 18);

                imshow(img)
                title('Undistorted Image with checkerboard axes');

                line(x1, y1, 'linewidth', 5, 'Color', 'green');
                line(x2, y2, 'linewidth', 5, 'Color', 'red');

            end

        end

        function gridCoords = screen2grid(self, screenCoords)
            % cam_tf = self.getCameraPose();
            cam_tf = self.camPose;
            R = cam_tf(1:3, 1:3);
            P = cam_tf(1:3, 4);

            gridCoords = pointsToWorld(self.params, R, P, screenCoords);
        end

        function worldCoords = screen2world(self, screenCoords)

            gc = self.screen2grid(screenCoords)
            absGc = [transpose(gc); 0; 1];
            worldCoords = self.transform * absGc;
            worldCoords = transpose(worldCoords(1:3));
        end

        function transform = camTransform(self, gridCoords)
            offset = [50 -100];
            dh_table = [0 0 50 deg2rad(180)
                    0 deg2rad(-90) -100 0];
            TB_I = dh2mat(dh_table(1, :));
            TI_G = dh2mat(dh_table(2, :));

            transform = TB_I * TI_G;
        end

        function ballData = detectBalls(self)
            [iniImage, newOrigin] = self.getImage();

            basedImg = abs(iniImage - self.baseImg);

            hsvImage = rgb2hsv(iniImage);

            theSat = hsvImage(:, :, 2);
            theSat_2 = theSat.^3;
            threshSat = graythresh(theSat_2);

            realBallsSat = theSat_2 > threshSat;
            
            theSat_two = theSat.^2;
            threshSatTwo = graythresh(theSat_two) * 1.0;
            threshSatTwoTwo = graythresh(theSat_two) * 1.1;
            twoBallsSat = theSat_two > threshSatTwo;
            twoTwoBallsSat = theSat_two > threshSatTwoTwo;



            disp("======== SAT THINGS =========")
            figure(25)
            imshow(twoBallsSat)
            figure(26)
            imshow(twoTwoBallsSat)

            disp("============ SATS ==========")
            disp("CUBE: " + threshSat + " SQUARE: " + threshSatTwoTwo)

            
            %% BlobAnalysis
            blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true, ...
                'CentroidOutputPort', true, ...
                'BoundingBoxOutputPort', true, ...
                'ExcludeBorderBlobs', true, ...
                'MinimumBlobArea', 3000)

            % [someA, centroids, bounding] = step(blobAnalysis, realBallsSat); % CUBED
            [someA, centroids, bounding] = step(blobAnalysis, twoBallsSat); % SQUARED

            someCircles = centroids;
            someCircles(:, 3) = 100;
            colorLabels = [];
            ballData = []

            for i = (1:size(centroids, 1))
                thisColor = self.getColor(impixel(iniImage, centroids(i, 1), centroids(i, 2)));
                disp(thisColor)
                thisColorName = self.getColorName(thisColor)
                colorLabels = [colorLabels; thisColorName];
                ballData = [ballData; centroids(i, 1), centroids(i, 2), uint32(thisColor)]
            end

            if self.DEBUG
                disp("===== BALL DATA =====")
                disp(ballData)
            end

            disp("PIXELS")
            % color1 = self.getColor(impixel(iniImage, centroids(1,1), centroids(1,2)))
            % color2 = self.getColor(impixel(iniImage, centroids(2,1), centroids(2,2)))
            % color3 = self.getColor(impixel(iniImage, centroids(3,1), centroids(3,2)))
            % color4 = self.getColor(impixel(iniImage, centroids(4,1), centroids(4,2)))
            % colorLabels = [color1;color2;color3;color4]

            imBallAnn = imresize(iniImage, 0.5);
            % imBallAnn = insertObjectAnnotation(imBallAnn, 'rectangle', 0.5 * bounding, colorLabels);
            imBallAnn = insertObjectAnnotation(imBallAnn, 'circle', 0.5 * someCircles, colorLabels);
            figure(15)
            imshow(imBallAnn)

        end

        function [img, newOrigin] = getImage(self)
            originalImage = self.cam.snapshot();
            [img, newOrigin] = undistortImage(originalImage, self.params, 'OutputView', 'full');

        end

        function returnColor = getColor(self, rgbvals)
            bestScore = Inf;
            returnColor = 0;

            for i = 1:4
                compareColor = self.ourColorRGBs(i, :);
                colorDiff = (compareColor/norm(compareColor)) - (rgbvals/norm(rgbvals));
                currScore = norm(colorDiff);
                % currScore = sqrt((rgbvals(1)-our_colors(i,1))^2 + (rgbvals(2)-our_colors(i,2))^2 + (rgbvals(3)-our_colors(i,3))^2)
                if currScore < bestScore
                    bestScore = currScore;
                    returnColor = cell2mat(self.ourColorEnums(i));
                    disp(" THIS IS THE RETURN COLOR" + returnColor)
                end

            end

        end

        function colorName = getColorName(self, aColor)

            colorName = self.ourColorNames(uint32(aColor))

        end

    end

end
