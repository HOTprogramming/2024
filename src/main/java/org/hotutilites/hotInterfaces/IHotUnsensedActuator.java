package org.hotutilites.hotInterfaces;

public interface IHotUnsensedActuator <RS, RC>
{
   public void performAction(RC commander, RS robotState);
}