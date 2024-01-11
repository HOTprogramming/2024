package org.hotutilites.hotInterfaces;

public interface IHotSensor<RS, T> 
{

    public void setRobotState(RS robotState);

    public void updateState();

    public void zeroSensor();

    public void setSensorValue(T value);
}