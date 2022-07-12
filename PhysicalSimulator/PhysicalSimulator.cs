using AdvancedTimers;
using Constants;
using EventArgsLibrary;
using MessagesNS;
using PerformanceMonitorTools;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Configuration;
using Utilities;

namespace PhysicalSimulatorNS
{
    public class PhysicalSimulator
    {

        string TeamIpAddress = TeamMulticastIP.TeamIpRCT;
        string OpponentTeamIpAddress = TeamMulticastIP.TeamIpTechUnited;

        double stadiumLength = 0;
        double fieldLength = 0;
        double stadiumWidth = 0;
        double fieldWidth = 0;
        double goalWidth = 0;

        ConcurrentDictionary<int, PhysicalRobotSimulator> robotList = new ConcurrentDictionary<int, PhysicalRobotSimulator>();
        object lockRobotList = new object();

        ConcurrentDictionary<int, PhysicalBallSimulator> ballSimulatedList = new ConcurrentDictionary<int, PhysicalBallSimulator>();
        double fSampling = 50;

        HighFreqTimerV2 highFrequencyTimer;

        ConcurrentDictionary<int, FiltreOrdre1> filterLowPassVxList = new ConcurrentDictionary<int, FiltreOrdre1>();
        ConcurrentDictionary<int, FiltreOrdre1> filterLowPassVyList = new ConcurrentDictionary<int, FiltreOrdre1>();
        ConcurrentDictionary<int, FiltreOrdre1> filterLowPassVThetaList = new ConcurrentDictionary<int, FiltreOrdre1>();

        public PhysicalSimulator(string typeTerrain)
        {
            switch (typeTerrain)
            {
                case "Cachan":
                    stadiumLength = 8;
                    stadiumWidth = 4;
                    break;
                case "RoboCup":
                    stadiumLength = RoboCupField.StadiumLength;
                    fieldLength = RoboCupField.FieldLength;
                    stadiumWidth = RoboCupField.StadiumWidth;
                    fieldWidth = RoboCupField.FieldWidth;
                    goalWidth = 2.4;
                    break;
                default:
                    break;
            }

            ballSimulatedList.TryAdd(0, new PhysicalBallSimulator(0, 0));
            //ballSimulatedList.Add(1, new PhysicalBallSimulator(3, 0));
            //ballSimulatedList.Add(2, new PhysicalBallSimulator(6, 0));


            highFrequencyTimer = new HighFreqTimerV2(fSampling, "PhysicalSimulator");
            highFrequencyTimer.Tick += HighFrequencyTimer_Tick;
            highFrequencyTimer.Start();
        }

        public void RegisterRobot(int id, double xpos, double yPos)
        {

            var physicalRobotSimu = new PhysicalRobotSimulator(xpos, yPos);
            lock (lockRobotList)
            {
                robotList.AddOrUpdate(id, physicalRobotSimu, (key, value) => physicalRobotSimu);
            }

            var filterLowPassVx = new FiltreOrdre1();
            filterLowPassVx.LowPassFilterInit(fSampling, 10);
            var filterLowPassVy = new FiltreOrdre1();
            filterLowPassVy.LowPassFilterInit(fSampling, 10);
            var filterLowPassVTheta = new FiltreOrdre1();
            filterLowPassVTheta.LowPassFilterInit(fSampling, 10);

            filterLowPassVxList.AddOrUpdate(id, filterLowPassVx, (key, value) => filterLowPassVx);
            filterLowPassVyList.AddOrUpdate(id, filterLowPassVy, (key, value) => filterLowPassVy);
            filterLowPassVThetaList.AddOrUpdate(id, filterLowPassVTheta, (key, value) => filterLowPassVTheta);
        }

        private void HighFrequencyTimer_Tick(object sender, EventArgs e)
        {
            /// Calcul des déplacements théoriques des robots avec gestion des collisions
            /// 
            /// On calcule les nouvelles positions théoriques de tous les robots si il n'y a pas collision
            lock (lockRobotList)
            {
                foreach (var robot in robotList)
                {
                    robot.Value.newXWithoutCollision = robot.Value.X + (robot.Value.VxRefRobot * Math.Cos(robot.Value.Theta) - robot.Value.VyRefRobot * Math.Sin(robot.Value.Theta)) / fSampling;
                    robot.Value.newYWithoutCollision = robot.Value.Y + (robot.Value.VxRefRobot * Math.Sin(robot.Value.Theta) + robot.Value.VyRefRobot * Math.Cos(robot.Value.Theta)) / fSampling;
                    robot.Value.newThetaWithoutCollision = robot.Value.Theta + robot.Value.Vtheta / fSampling;
                }

                //TODO : Gérer les collisions polygoniales en déclenchant l'étude fine à l'aide d'un cercle englobant.
                //TODO : gérer la balle et les rebonds robots poteaux cages
                //TODO : gérer la perte d'énergie de la balle : modèle à trouver... mesure précise :) faite sur le terrain : 1m.s-1 -> arrêt à 10m
                //TODO : gérer le tir (ou passe)
                //TODO : gérer les déplacements balle au pied
                //TODO : gérer les cas de contestation

                /// On Initialise les collisions robots à false
                foreach (var robot in robotList)
                {
                    robot.Value.Collision = false;
                }

                /// Pour chacun des robots, on regarde les collisions avec les murs
                foreach (var robot in robotList)
                {
                    /// On check les murs 
                    if ((robot.Value.newXWithoutCollision + robot.Value.radius > stadiumLength / 2) || (robot.Value.newXWithoutCollision - robot.Value.radius < -stadiumLength / 2)
                        || (robot.Value.newYWithoutCollision + robot.Value.radius > stadiumWidth / 2) || (robot.Value.newYWithoutCollision - robot.Value.radius < -stadiumWidth / 2))
                    {
                        robot.Value.Collision = true;
                    }
                }

                /// Pour chacun des robots, on regarde les collisions avec les autres robots
                foreach (var robot in robotList)
                {
                    //On check les autres robots
                    foreach (var otherRobot in robotList)
                    {
                        if (otherRobot.Key != robot.Key) //On exclu le test entre robots identiques
                        {
                            if (Toolbox.Distance(robot.Value.newXWithoutCollision, robot.Value.newYWithoutCollision, otherRobot.Value.newXWithoutCollision, otherRobot.Value.newYWithoutCollision) < robot.Value.radius * 2)
                                robot.Value.Collision = true;
                        }
                    }
                }
            }

            /// On calcule la nouvelle position théorique de la balle si il n'y a pas de collision
            foreach (var ballSimu in ballSimulatedList)
            {
                var ballSimulated = ballSimu.Value;
                ballSimulated.positionForced = false;
                ballSimulated.newX = ballSimulated.X + ballSimulated.VxRefTerrain / fSampling;
                ballSimulated.newY = ballSimulated.Y + ballSimulated.VyRefTerrain / fSampling;
            }

            /// On initialise les collisions balles à false
            foreach (var ballSimu in ballSimulatedList)
            {
                ballSimu.Value.Collision = false;
            }

            /// On check les collisions balle-robot
            lock (lockRobotList)
            {
                foreach (var robot in robotList)
                {
                    foreach (var ballSimu in ballSimulatedList)
                    {
                        ///// Par défaut, on met le statut de prise de balle du robot à false
                        //robot.Value.IsHandlingBall = false;

                        /// On gère les prises de balles simulées ici : Si la balle touche un robot, soit elle rebondit soit elle est prise
                        /// Si l'angle entre l'orientation du robot le vecteur robot balle est inférieur en valeur absolue à 30°, le robot prend la balle
                        /// Sinon elle rebondit
                        /// 

                        /// Si un tir est demandé
                        if (robot.Value.IsShootRequested)
                        {
                            robot.Value.IsShootRequested = false;
                            if (ballSimu.Value.isHandledByRobot)
                            {
                                ballSimu.Value.VxRefTerrain = robot.Value.VxRefRobot * Math.Cos(robot.Value.Theta) - robot.Value.VyRefRobot * Math.Sin(robot.Value.Theta)
                                    + robot.Value.ShootingSpeed * Math.Cos(robot.Value.Theta);
                                ballSimu.Value.VyRefTerrain = robot.Value.VxRefRobot * Math.Sin(robot.Value.Theta) + robot.Value.VyRefRobot * Math.Cos(robot.Value.Theta)
                                    + robot.Value.ShootingSpeed * Math.Sin(robot.Value.Theta);
                                ballSimu.Value.isHandledByRobot = false;
                                robot.Value.IsHandlingBall = false;
                            }
                        }

                        /// Si pas de tir demandé
                        else if (!ballSimu.Value.isHandledByRobot)  /// On teste si la balle est déjà prise par un robot ou pas
                        {
                            /// Si la balle est en contact avec un robot
                            if (Toolbox.Distance(robot.Value.newXWithoutCollision, robot.Value.newYWithoutCollision, ballSimu.Value.newX, ballSimu.Value.newY) < 1 * (robot.Value.radius + ballSimu.Value.radius))
                            {
                                double angleRobotBalle = Math.Atan2(ballSimu.Value.Y - robot.Value.Y, ballSimu.Value.X - robot.Value.X);
                                angleRobotBalle = Toolbox.ModuloByAngle(robot.Value.Theta, angleRobotBalle);

                                if (Math.Abs(angleRobotBalle - robot.Value.Theta) < Toolbox.DegToRad(30))
                                {
                                    Console.WriteLine("Prise de balle par un robot");
                                    //ballSimu.Value.Vx = robot.Value.VxRefRobot;
                                    //ballSimu.Value.Vy = robot.Value.VyRefRobot;
                                    ballSimu.Value.isHandledByRobot = true;
                                    ballSimu.Value.lastRobotIdHavingBall = robot.Key;
                                    robot.Value.IsHandlingBall = true;
                                }
                                else
                                {
                                    Console.WriteLine("Rebond de balle sur un robot");
                                    ballSimu.Value.Collision = true;
                                    ballSimu.Value.VxRefTerrain = +robot.Value.VxRefRobot * Math.Cos(robot.Value.Theta) - robot.Value.VyRefRobot * Math.Sin(robot.Value.Theta) - 0.8 * ballSimu.Value.VxRefTerrain;
                                    ballSimu.Value.VyRefTerrain = +robot.Value.VxRefRobot * Math.Sin(robot.Value.Theta) + robot.Value.VyRefRobot * Math.Cos(robot.Value.Theta) - 0.8 * ballSimu.Value.VyRefTerrain;
                                    ballSimu.Value.lastRobotIdHavingBall = robot.Key;
                                    ballSimu.Value.isHandledByRobot = false;
                                }
                            }
                            else
                            {
                                ballSimu.Value.isHandledByRobot = false;
                            }
                        }
                    }
                }
            }

            ///On check les sorties de balle en touche
            foreach (var ballSimu in ballSimulatedList)
            {
                ///On regarde si la balle sort en touche
                if ((ballSimu.Value.newY - ballSimu.Value.radius > fieldWidth / 2) || (ballSimu.Value.newY + ballSimu.Value.radius < -fieldWidth / 2))
                {
                    //if (ballSimu.Value.isInField)
                    {
                        /// On libère la balle de tous les robots
                        foreach (var robot in robotList)
                        {
                            robot.Value.IsHandlingBall = false;
                        }
                        ballSimu.Value.isHandledByRobot = false;

                        /// Si on est en jeu, on arrêt la balle et on lance un event d'arrêt, puis un event de Throw-in
                        ballSimu.Value.VxRefTerrain = 0; //On simule un arrêt de la balle
                        ballSimu.Value.VyRefTerrain = 0; //On simule un arrêt de la balle

                        /// On gère les sorties de balle en touche en replaçant la balle sur la ligne de touche
                        if (ballSimu.Value.newY - ballSimu.Value.radius > fieldWidth / 2)
                            ballSimu.Value.newY = fieldWidth / 2 - 0.1;

                        if (ballSimu.Value.newY + ballSimu.Value.radius < -fieldWidth / 2)
                            ballSimu.Value.newY = -fieldWidth / 2 + 0.1;

                        ballSimu.Value.ForceBallPosition(ballSimu.Value.newX, ballSimu.Value.newY);

                        /// On fabrique un message RefBox de type ThrowIn
                        RefBoxMessage rbMsg = new RefBoxMessage();
                        rbMsg.command = RefBoxCommand.THROWIN;

                        if (ballSimu.Value.lastRobotIdHavingBall / 10 * 10 == (int)TeamId.Team1)
                            rbMsg.targetTeam = TeamMulticastIP.TeamIpTechUnited;
                        else
                            rbMsg.targetTeam = TeamMulticastIP.TeamIpRCT;

                        OnRefBoxCommand(rbMsg);
                        ballSimu.Value.isInField = false;
                    }
                }

                /// On regarde si la balle sort en corner, en sortie de but ou bien il y a but
                if ((ballSimu.Value.newX - ballSimu.Value.radius > fieldLength / 2) || (ballSimu.Value.newX + ballSimu.Value.radius < -fieldLength / 2))
                {
                    /// On libère la balle de tous les robots
                    foreach (var robot in robotList)
                    {
                        robot.Value.IsHandlingBall = false;
                    }
                    ballSimu.Value.isHandledByRobot = false;

                    /// Si on est en jeu, on arrêt la balle et on lance un event d'arrêt, puis un event de type dépendant de la situation                        
                    ballSimu.Value.VxRefTerrain = 0;
                    ballSimu.Value.VyRefTerrain = 0;
                    ballSimu.Value.isInField = false;

                    ///Si la balle est dans le but
                    if ((ballSimu.Value.newY < goalWidth / 2) && (ballSimu.Value.newY > -goalWidth / 2))
                    {
                        RefBoxMessage rbMsg = new RefBoxMessage();
                        rbMsg.command = RefBoxCommand.GOAL;
                        if (ballSimu.Value.newX > 0)
                            rbMsg.targetTeam = TeamMulticastIP.TeamIpRCT;
                        else
                            rbMsg.targetTeam = TeamMulticastIP.TeamIpTechUnited;

                        OnRefBoxCommand(rbMsg);
                        ///On ramène la balle au centre du terrain
                        ballSimu.Value.ForceBallPosition(0, 0);
                    }
                    else ///La balle est en corner ou en sortie de but
                    {
                        RefBoxMessage rbMsg = new RefBoxMessage();
                        if (ballSimu.Value.newX > 0 && ballSimu.Value.lastRobotIdHavingBall / 10 * 10 == (int)TeamId.Team1)
                        {
                            rbMsg.command = RefBoxCommand.GOALKICK;
                            rbMsg.targetTeam = TeamMulticastIP.TeamIpTechUnited;

                            if (ballSimu.Value.Y >= 0)
                                ballSimu.Value.ForceBallPosition(fieldLength / 2 - 0.75, 3.9 / 2);
                            else
                                ballSimu.Value.ForceBallPosition(fieldLength / 2 - 0.75, -3.9 / 2);
                        }
                        else if (ballSimu.Value.newX > 0 && ballSimu.Value.lastRobotIdHavingBall / 10 * 10 == (int)TeamId.Team2)
                        {
                            rbMsg.command = RefBoxCommand.CORNER;
                            rbMsg.targetTeam = TeamMulticastIP.TeamIpRCT;
                            if (ballSimu.Value.Y >= 0)
                                ballSimu.Value.ForceBallPosition(fieldLength / 2, fieldWidth / 2);
                            else
                                ballSimu.Value.ForceBallPosition(fieldLength / 2, -fieldWidth / 2);
                        }
                        if (ballSimu.Value.newX < 0 && ballSimu.Value.lastRobotIdHavingBall / 10 * 10 == (int)TeamId.Team1)
                        {
                            rbMsg.command = RefBoxCommand.CORNER;
                            rbMsg.targetTeam = TeamMulticastIP.TeamIpTechUnited;
                            if (ballSimu.Value.Y >= 0)
                                ballSimu.Value.ForceBallPosition(-fieldLength / 2, fieldWidth / 2);
                            else
                                ballSimu.Value.ForceBallPosition(-fieldLength / 2, -fieldWidth / 2);
                        }
                        if (ballSimu.Value.newX < 0 && ballSimu.Value.lastRobotIdHavingBall / 10 * 10 == (int)TeamId.Team2)
                        {
                            rbMsg.command = RefBoxCommand.GOALKICK;
                            rbMsg.targetTeam = TeamMulticastIP.TeamIpRCT;

                            if (ballSimu.Value.Y >= 0)
                                ballSimu.Value.ForceBallPosition(-fieldLength / 2 + 0.75, 3.9 / 2);
                            else
                                ballSimu.Value.ForceBallPosition(-fieldLength / 2 + 0.75, -3.9 / 2);
                        }
                        OnRefBoxCommand(rbMsg);
                    }
                }

                /// On regarde si la balle est en jeu
                if (ballSimu.Value.positionForced 
                    || (ballSimu.Value.newX - ballSimu.Value.radius > fieldLength / 2)
                    || (ballSimu.Value.newX + ballSimu.Value.radius < -fieldLength / 2)
                    || (ballSimu.Value.newY - ballSimu.Value.radius > fieldWidth / 2)
                    || (ballSimu.Value.newY + ballSimu.Value.radius < -fieldWidth / 2))
                {
                    ///Ball out of field
                    ;
                }
                else
                {
                    ballSimu.Value.isInField = true;
                }
            }

            //On check les collisions balle-murs
            foreach (var ballSimu in ballSimulatedList)
            {
                // On ne le fait que si on n'a pas eu de forçage de position avant
                if (!ballSimu.Value.positionForced)
                {
                    //Gestion des collisions balle-murs
                    //On check les murs virtuels
                    //Mur haut ou bas
                    if ((ballSimu.Value.newY - ballSimu.Value.radius > fieldWidth / 2) || (ballSimu.Value.newY + ballSimu.Value.radius < -fieldWidth / 2))
                    {
                        ballSimu.Value.VxRefTerrain = 0; //On simule un arrêt
                        ballSimu.Value.VyRefTerrain = 0; //On simule un arrêt
                    }
                    //Mur gauche ou droit
                    if ((ballSimu.Value.newX - ballSimu.Value.radius > fieldLength / 2) || (ballSimu.Value.newX + ballSimu.Value.radius < -fieldLength / 2))
                    {
                        ballSimu.Value.VxRefTerrain = 0; //On simule un arrêt
                        ballSimu.Value.VyRefTerrain = 0; //On simule un arrêt
                    }
                }
            }

            //Gestion de la décélération de la balle
            //double deceleration = 0.5;


            //Calcul de la nouvelle Location des robots
            lock (lockRobotList)
            {
                foreach (var robot in robotList)
                {
                    if (!robot.Value.Collision)
                    {
                        robot.Value.X = robot.Value.newXWithoutCollision;
                        robot.Value.Y = robot.Value.newYWithoutCollision;
                        robot.Value.Theta = robot.Value.newThetaWithoutCollision;
                    }
                    else
                    {
                        robot.Value.VxRefRobot = 0;
                        robot.Value.VyRefRobot = 0;
                        robot.Value.Vtheta = 0;
                    }

                    //Emission d'un event de position physique 
                    Location loc = new Location(robot.Value.X, robot.Value.Y, robot.Value.Theta, robot.Value.VxRefRobot, robot.Value.VyRefRobot, robot.Value.Vtheta);
                    OnPhysicalRobotLocation(robot.Key, loc);
                    OnPhysicalBallHandling(robot.Key, robot.Value.IsHandlingBall);

                    /// Pour le debug
                    PhysicalSimulatorMonitor.PhysicalSimulatorReceived();
                }

                //Calcul de la nouvelle location des balles
                List<LocationExtended> newBallLocationList = new List<LocationExtended>();
                foreach (var ballSimu in ballSimulatedList)
                {
                    if (!ballSimu.Value.isHandledByRobot)
                    {
                        /// La balle n'est pas controlée par le robot
                        ballSimu.Value.newX = ballSimu.Value.X + ballSimu.Value.VxRefTerrain / fSampling;
                        ballSimu.Value.newY = ballSimu.Value.Y + ballSimu.Value.VyRefTerrain / fSampling;

                        /// On vérifie que la balle ne soit pas incluse dans un robot
                        /// Si c'est le cas, on la décale en périphérie.
                        /// 
                        foreach (var robot in robotList)
                        {
                            if (Toolbox.Distance(robot.Value.X, robot.Value.Y, ballSimu.Value.newX, ballSimu.Value.newY) < 1 * (robot.Value.radius + ballSimu.Value.radius))
                            {
                                double angleRobotBalle = Math.Atan2(ballSimu.Value.newY - robot.Value.Y, ballSimu.Value.newX - robot.Value.X);
                                ballSimu.Value.newX = robot.Value.X + (robot.Value.radius + ballSimu.Value.radius) * Math.Cos(angleRobotBalle);
                                ballSimu.Value.newY = robot.Value.Y + (robot.Value.radius + ballSimu.Value.radius) * Math.Sin(angleRobotBalle);
                            }
                        }

                        ballSimu.Value.X = ballSimu.Value.newX;
                        ballSimu.Value.Y = ballSimu.Value.newY;

                        ballSimu.Value.VxRefTerrain = ballSimu.Value.VxRefTerrain * 0.999;
                        ballSimu.Value.VyRefTerrain = ballSimu.Value.VyRefTerrain * 0.999;

                        newBallLocationList.Add(new LocationExtended(ballSimu.Value.X, ballSimu.Value.Y, 0, ballSimu.Value.VxRefTerrain, ballSimu.Value.VyRefTerrain, 0, ObjectType.Balle));
                    }
                    else
                    {
                        /// La balle est controlée par le robot
                        /// Sa position est celle du robot décalée
                        var robotControlling = robotList[ballSimu.Value.lastRobotIdHavingBall];
                        ballSimu.Value.X = robotControlling.X + (robotControlling.radius + ballSimu.Value.radius) * Math.Cos(robotControlling.Theta);
                        ballSimu.Value.Y = robotControlling.Y + (robotControlling.radius + ballSimu.Value.radius) * Math.Sin(robotControlling.Theta);
                        ballSimu.Value.VxRefTerrain = robotControlling.VxRefRobot * Math.Cos(robotControlling.Theta) - robotControlling.VyRefRobot * Math.Sin(robotControlling.Theta);
                        ballSimu.Value.VyRefTerrain = robotControlling.VyRefRobot * Math.Sin(robotControlling.Theta) + robotControlling.VyRefRobot * Math.Cos(robotControlling.Theta); ;
                        newBallLocationList.Add(new LocationExtended(ballSimu.Value.X, ballSimu.Value.Y, 0, ballSimu.Value.VxRefTerrain, ballSimu.Value.VyRefTerrain, 0, ObjectType.Balle));
                    }
                }
                OnPhysicalBallListPosition(newBallLocationList);

                ConcurrentBag<LocationExtended> objectsLocationList = new ConcurrentBag<LocationExtended>();
                foreach (var robot in robotList)
                {
                    //if((robot.Key/10)*10 == (int)TeamId.Team1)
                    //    objectsLocationList.Add(new LocationExtended(robot.Value.X, robot.Value.Y, robot.Value.Theta, robot.Value.VxRefRobot, robot.Value.VyRefRobot, robot.Value.Vtheta, ObjectType.RobotTeam1));
                    //else
                    //    objectsLocationList.Add(new LocationExtended(robot.Value.X, robot.Value.Y, robot.Value.Theta, robot.Value.VxRefRobot, robot.Value.VyRefRobot, robot.Value.Vtheta, ObjectType.RobotTeam2));

                    /// Renvoie des objets perçus sans donner leur type 
                    /// Les algos d'identification doivent ensuite le trouver dans la perception
                    objectsLocationList.Add(new LocationExtended(robot.Value.X, robot.Value.Y, robot.Value.Theta, robot.Value.VxRefRobot, robot.Value.VyRefRobot, robot.Value.Vtheta, ObjectType.ObstacleLidar));
                }
                OnPhysicalObjectListLocation(objectsLocationList);
            }
        }

        public void OnForceSimulatorEventReceived(object sender, SimulatorActionArgs e)
        {            
            switch(e.action)
            {
                case StoppedGameAction.PENALTY:
                    ballSimulatedList[0].X = fieldLength / 2 - 2.4;
                    ballSimulatedList[0].Y = 0;
                    break;
                case StoppedGameAction.CORNER:
                    ballSimulatedList[0].X = fieldLength / 2;
                    ballSimulatedList[0].Y = fieldWidth / 2;
                    break;
                case StoppedGameAction.GOALKICK:
                    ballSimulatedList[0].X = - fieldLength / 2 + 1;
                    ballSimulatedList[0].Y = 1.25;
                    break;
                case StoppedGameAction.THROWIN:
                    ballSimulatedList[0].X = fieldLength / 4;
                    ballSimulatedList[0].Y = fieldWidth / 2;
                    break;
                default:
                    break;
            }
            ballSimulatedList[0].VxRefTerrain = 0;
            ballSimulatedList[0].VyRefTerrain = 0;

        }

        Configuration configFile;
        public void OnConfigUpdatedReceived(object sender, ConfigurationArgs e)
        {
            configFile = e.config;

            fieldLength = 22;
            if (configFile.AppSettings.Settings["FieldLength"] != null)
                double.TryParse(configFile.AppSettings.Settings["FieldLength"].Value, out fieldLength);
            fieldWidth = 14;
            if (configFile.AppSettings.Settings["FieldWidth"] != null)
                double.TryParse(configFile.AppSettings.Settings["FieldWidth"].Value, out fieldWidth);

            stadiumLength = fieldLength + 2;
            stadiumWidth = fieldWidth + 2;
        }

        public void SetRobotSpeed(object sender, PolarSpeedArgs e)
        {
            //Attention, les vitesses proviennent de l'odométrie et sont donc dans le référentiel robot

            lock (lockRobotList)
            {
                if (robotList.ContainsKey(e.RobotId))
                {
                    robotList[e.RobotId].VxRefRobot = filterLowPassVxList[e.RobotId].Filter(e.Vx);
                    robotList[e.RobotId].VyRefRobot = filterLowPassVyList[e.RobotId].Filter(e.Vy);
                    robotList[e.RobotId].Vtheta = filterLowPassVThetaList[e.RobotId].Filter(e.Vtheta);
                }
            }
        }

        public void SetRobotPosition(int id, double x, double y, double theta)
        {
            //Attention, les positions sont dans le référentiel terrain
            lock (lockRobotList)
            {
                if (robotList.ContainsKey(id))
                {
                    robotList[id].X = x;
                    robotList[id].Y = y;
                    robotList[id].Theta = theta;
                }
            }
        }

        public void RequestRobotShoot(int id, double shootingSpeed)
        {
            lock (lockRobotList)
            {
                if (robotList.ContainsKey(id))
                {
                    robotList[id].IsShootRequested = true;
                    robotList[id].ShootingSpeed = shootingSpeed;
                }
            }
        }

        public void OnCollisionReceived(object sender, EventArgsLibrary.CollisionEventArgs e)
        {
            SetRobotPosition(e.RobotId, e.RobotRealPositionRefTerrain.X, e.RobotRealPositionRefTerrain.Y, e.RobotRealPositionRefTerrain.Theta);
        }
        public void OnShootOrderReceived(object sender, EventArgsLibrary.ShootEventArgs e)
        {
            RequestRobotShoot(e.RobotId, e.shootingSpeed);
        }

        //Output events
        public event EventHandler<RefBoxMessageArgs> OnSimulatorRefboxCommandEvent;
        public virtual void OnRefBoxCommand(RefBoxMessage msg)
        {
            var handler = OnSimulatorRefboxCommandEvent;
            if (handler != null)
            {
                handler(this, new RefBoxMessageArgs{refBoxMsg= msg});
            }
        }
        //public event EventHandler<DataReceivedArgs> OnMulticastSendRefBoxCommandEvent;
        //public virtual void OnMulticastSendRefBoxCommand(byte[] data)
        //{
        //    var handler = OnMulticastSendRefBoxCommandEvent;
        //    if (handler != null)
        //    {
        //        handler(this, new DataReceivedArgs { Data = data });
        //    }
        //}

        public event EventHandler<LocationArgs> OnPhysicalRobotLocationEvent;
        public virtual void OnPhysicalRobotLocation(int id, Location location)
        {
            var handler = OnPhysicalRobotLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationArgs { RobotId = id, Location = location });
            }
        }

        public event EventHandler<BallHandlingSensorArgs> OnPhysicalBallHandlingEvent;
        public virtual void OnPhysicalBallHandling(int id, bool isHandling)
        {
            var handler = OnPhysicalBallHandlingEvent;
            if (handler != null)
            {
                handler(this, new BallHandlingSensorArgs { RobotId = id,  IsHandlingBall = isHandling});
            }
        }

        public event EventHandler<LocationExtendedListArgs> OnPhysicalBallPositionListEvent;
        public virtual void OnPhysicalBallListPosition(List<LocationExtended> locationList)
        {
            var handler = OnPhysicalBallPositionListEvent;
            if (handler != null)
            {
                handler(this, new LocationExtendedListArgs { LocationExtendedList = locationList });
            }
        }

        public delegate void ObjectsPositionEventHandler(object sender, LocationExtendedConcurrentBagArgs e);
        public event EventHandler<LocationExtendedConcurrentBagArgs> OnPhysicicalObjectListLocationEvent;
        public virtual void OnPhysicalObjectListLocation(ConcurrentBag<LocationExtended> locationList)
        {
            var handler = OnPhysicicalObjectListLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationExtendedConcurrentBagArgs { LocationExtendedList = locationList });
            }
        }
    }

    public class PhysicalRobotSimulator
    {
        public double radius = 0.25;
        public double X;
        public double Y;
        public double Theta;

        public double newXWithoutCollision;
        public double newYWithoutCollision;
        public double newThetaWithoutCollision;

        public double VxRefRobot;
        public double VyRefRobot;
        public double Vtheta;

        public bool Collision;
        public bool IsHandlingBall;
        public bool IsShootRequested;
        public double ShootingSpeed;



        public PhysicalRobotSimulator(double xPos, double yPos)
        {
            X = xPos;
            Y = yPos;
            IsHandlingBall = false;
        }
    }

    public class PhysicalBallSimulator
    {
        public double radius = 0.115;
        public double X;
        public double Y;
        public double Z;
        public bool positionForced = false;
        //public double Theta;

        public double newX;
        public double newY;
        //public double newThetaWithoutCollision;

        public double VxRefTerrain;
        public double VyRefTerrain;
        //public double Vtheta;

        public bool Collision;
        public bool isHandledByRobot = false;
        public int lastRobotIdHavingBall;
        public bool isInField = true;


        public PhysicalBallSimulator(double xPos, double yPos)
        {
            X = xPos;
            Y = yPos;
        }

        public void ForceBallPosition(double xPos, double yPos)
        {
            X = xPos;
            Y = yPos;
            newX = xPos;
            newY = yPos;
            positionForced = true;
        }

    }
}
