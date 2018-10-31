within ;
package TutorialDCMotor
  model MotorModel
    Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5)
      annotation (Placement(transformation(extent={{-56,30},{-36,50}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-76,-38},{-56,-18}})));
    Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.05)
      annotation (Placement(transformation(extent={{-18,30},{2,50}})));
    Modelica.Electrical.Analog.Basic.EMF emf
      annotation (Placement(transformation(extent={{6,-18},{26,2}})));
    Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-74,6})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
      annotation (Placement(transformation(extent={{58,-18},{78,2}})));
    Modelica.Blocks.Interfaces.RealInput u
      annotation (Placement(transformation(extent={{-126,-14},{-86,26}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
      "Right flange of shaft"
      annotation (Placement(transformation(extent={{90,-16},{110,4}})));
  equation
    connect(signalVoltage.n, resistor.p)
      annotation (Line(points={{-74,16},{-74,40},{-56,40}}, color={0,0,255}));
    connect(resistor.n, inductor.p)
      annotation (Line(points={{-36,40},{-18,40}}, color={0,0,255}));
    connect(ground.p, signalVoltage.p) annotation (Line(points={{-66,-18},{-74,
            -18},{-74,-4}}, color={0,0,255}));
    connect(inductor.n, emf.p) annotation (Line(points={{2,40},{14,40},{14,2},{
            16,2}}, color={0,0,255}));
    connect(ground.p, emf.n)
      annotation (Line(points={{-66,-18},{16,-18}}, color={0,0,255}));
    connect(emf.flange, inertia.flange_a)
      annotation (Line(points={{26,-8},{58,-8}}, color={0,0,0}));
    connect(signalVoltage.v, u)
      annotation (Line(points={{-81,6},{-106,6}}, color={0,0,127}));
    connect(inertia.flange_b, flange) annotation (Line(points={{78,-8},{90,-8},
            {90,-6},{100,-6}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end MotorModel;

  model MotorDrive
    MotorModel motorModel
      annotation (Placement(transformation(extent={{32,-2},{52,18}})));
    Modelica.Blocks.Continuous.PID PID
      annotation (Placement(transformation(extent={{-14,-8},{6,12}})));
    Modelica.Blocks.Sources.Step step
      annotation (Placement(transformation(extent={{-76,-14},{-56,6}})));
    Modelica.Blocks.Math.Feedback feedback
      annotation (Placement(transformation(extent={{-44,-10},{-24,10}})));
    Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={82,-46})));
    Modelica.Mechanics.Rotational.Components.IdealGear idealGear
      annotation (Placement(transformation(extent={{66,-2},{86,18}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
      annotation (Placement(transformation(extent={{94,-2},{114,18}})));
  equation
    connect(motorModel.flange, idealGear.flange_a) annotation (Line(points={{52,
            7.4},{60,7.4},{60,8},{66,8}}, color={0,0,0}));
    connect(inertia.flange_a, idealGear.flange_b)
      annotation (Line(points={{94,8},{86,8}}, color={0,0,0}));
    connect(PID.y, motorModel.u) annotation (Line(points={{7,2},{24,2},{24,8.6},
            {31.4,8.6}}, color={0,0,127}));
    connect(feedback.y, PID.u) annotation (Line(points={{-25,0},{-25,-1},{-16,
            -1},{-16,2}}, color={0,0,127}));
    connect(feedback.u1, step.y) annotation (Line(points={{-42,0},{-48,0},{-48,
            -4},{-55,-4}}, color={0,0,127}));
    connect(feedback.u2, angleSensor.phi) annotation (Line(points={{-34,-8},{
            -36,-8},{-36,-46},{71,-46}}, color={0,0,127}));
    connect(angleSensor.flange, inertia.flange_b) annotation (Line(points={{92,
            -46},{110,-46},{110,8},{114,8}}, color={0,0,0}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{120,
              120}})),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              120,120}})),
      Documentation(info="<html>
<p>This is a motor drive model</p>
</html>"));
  end MotorDrive;
  annotation (uses(Modelica(version="3.2.2")));
end TutorialDCMotor;
