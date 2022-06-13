/*

Copyright (c) 2022, Arvid Norberg
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*/

#include "simulator/simulator.hpp"
#include "simulator/utils.hpp"

#include "test.hpp"
#include "create_torrent.hpp"
#include "settings.hpp"
#include "setup_swarm.hpp"
#include "utils.hpp"
#include "test_utils.hpp"
#include "setup_transfer.hpp" // for addr()
#include "disk_io.hpp"

#include "libtorrent/add_torrent_params.hpp"
#include "libtorrent/alert_types.hpp"

template <typename Setup, typename HandleAlerts>
void run_test(
	Setup setup
	, HandleAlerts on_alert
	, test_disk const downloader_disk_constructor = test_disk()
	, test_disk const seed_disk_constructor = test_disk()
	)
{
	char const* peer0_ip = "50.0.0.1";
	char const* peer1_ip = "50.0.0.2";

	lt::address peer0 = addr(peer0_ip);
	lt::address peer1 = addr(peer1_ip);

	// setup the simulation
	sim::default_config network_cfg;
	sim::simulation sim{network_cfg};
	sim::asio::io_context ios0 { sim, peer0 };
	sim::asio::io_context ios1 { sim, peer1 };

	lt::session_proxy zombie[2];

	lt::session_params params;
	// setup settings pack to use for the session (customization point)
	lt::settings_pack& pack = params.settings;
	pack = settings();

	pack.set_str(lt::settings_pack::listen_interfaces, make_ep_string(peer0_ip, false, "6881"));

	// create session
	std::shared_ptr<lt::session> ses[2];

	// session 0 is a downloader, session 1 is a seed

	params.disk_io_constructor = downloader_disk_constructor;
	ses[0] = std::make_shared<lt::session>(params, ios0);

	pack.set_str(lt::settings_pack::listen_interfaces, make_ep_string(peer1_ip, false, "6881"));

	params.disk_io_constructor = seed_disk_constructor.set_files(existing_files_mode::full_valid);
	ses[1] = std::make_shared<lt::session>(params, ios1);

	setup(*ses[0], *ses[1]);

	// only monitor alerts for session 0 (the downloader)
	print_alerts(*ses[0], [=](lt::session& ses, lt::alert const* a) {
		if (auto ta = lt::alert_cast<lt::add_torrent_alert>(a))
			ta->handle.connect_peer(lt::tcp::endpoint(peer1, 6881));
		on_alert(ses, a);
	}, 0);

	print_alerts(*ses[1], [](lt::session&, lt::alert const*){}, 1);

	sim::timer t(sim, lt::seconds(10), [&](boost::system::error_code const&)
	{
		// shut down
		int idx = 0;
		for (auto& s : ses)
		{
			zombie[idx++] = s->abort();
			s.reset();
		}
	});

	sim.run();
}

TORRENT_TEST(merge_hybrid_torrent)
{
	run_test([](lt::session& ses0, lt::session& ses1) {
		lt::add_torrent_params atp = ::create_test_torrent(10, lt::create_flags_t{}, 2);
		atp.flags &= ~lt::torrent_flags::auto_managed;
		atp.flags &= ~lt::torrent_flags::paused;

		// add the complete torrent to the seed
		ses1.async_add_torrent(atp);

		lt::info_hash_t const ih = atp.ti->info_hashes();

		// add v1-only magnet link
		atp.ti.reset();
		atp.info_hashes.v1 = ih.v1;
		ses0.async_add_torrent(atp);

		// add v2-only magnet link
		atp.info_hashes.v1.clear();
		atp.info_hashes.v2 = ih.v2;
		ses0.async_add_torrent(atp);
	},
	[](lt::session&, lt::alert const*) {}
	);
}
