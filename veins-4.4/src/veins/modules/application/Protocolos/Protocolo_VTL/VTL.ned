//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

package org.car2x.veins.modules.application.Protocolos.Protocolo_VTL;
import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;

simple VTL extends BaseWaveApplLayer
{
    parameters:
    double tiempo_semaforo;
    double shared_data_radio;
    double lider_selection_radio;

    @class(VTL);
    @display("i=block/app2");
    bool sendWhileParking  = default(false); //send messages when the car is parked
}
