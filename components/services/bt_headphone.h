/*
 *  Send audio to a bluetooth headset when it is there, to the dac when it is not
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#pragma once

/*
Keeps bluetooth available whichever output is in use, so the player notices its headset
arriving and leaving, and follows it.

The output backend is chosen once, from the "-o" argument, when squeezelite starts, and
squeezelite cannot be started twice - so following the headset means rewriting that
argument and restarting. One restart per transition, which for a headset that is put on
and taken off again is two per listening session.

Deliberately no inquiry scanning. A headset that has been paired once does not answer an
inquiry when merely switched on; it goes looking for its own last source instead. Scanning
for it would therefore not find it, while monopolising a radio that wifi also needs. So
the player stays connectable and lets the headset do the finding, and reconnects by
address once it knows one.

Configured with "bt_headphone" in nvs: the name of the headset, empty to switch all of
this off. Pairing itself still happens the usual way, with the headset in pairing mode
and the player started with "-o BT".
*/
void bt_headphone_svc_init(void);

/*
Where the audio currently goes: 1 for the headset, 0 for the dac, -1 when no headset is
configured and the question does not arise. Follows the headset rather than the output
squeezelite was started with, so it does not flicker during the reboot that a change
costs - see the definition for why.
*/
int bt_headphone_channel(void);
