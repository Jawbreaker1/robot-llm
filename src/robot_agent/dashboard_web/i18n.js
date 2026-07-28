((global) => {
  "use strict";

  const sv = Object.freeze({
    "common.missing": "—",
    "common.unknown": "okänd",
    "common.none": "ingen",
    "locale.swedish": "Svenska",
    "locale.english": "English",
    "locale.label": "Språk",
    "locale.selector.aria_label": "Välj språk",
    "bodies.controller.controller_id": "Styrenhets-ID",
    "bodies.controller.eyebrow": "Styrenhetsdetaljer",
    "bodies.controller.heartbeat": "Hjärtslag",
    "bodies.controller.instance_id": "Instans-ID",
    "bodies.controller.physical_capabilities": "Fysiska funktioner",
    "bodies.controller.state": "Tillstånd",
    "bodies.controller.state_version": "Tillståndsversion",
    "bodies.description": "Styrenheter exekverar. Kameror och mikrofoner observerar. De är olika typer av noder.",
    "bodies.eyebrow": "Distribuerad robotkropp",
    "bodies.logical.eyebrow": "Logisk kropp",
    "bodies.nodes.controller_transport": "styrenhet · USB/SSH",
    "bodies.nodes.front_camera": "Frontkamera",
    "bodies.nodes.future_perception": "framtida perceptionskälla",
    "bodies.nodes.microphone_array": "Mikrofonmatris",
    "bodies.nodes.robot_waiting": "robot_id väntar på registret",
    "bodies.read_only": "Skrivskyddad inventering",
    "bodies.safety.body": "Att en styrenhet blir synlig innebär aldrig automatiskt att rörelse är tillåten.",
    "bodies.safety.title": "Ingen exekveringsväg",
    "bodies.status.configured": "konfigurerad",
    "bodies.status.locked_plural": "Låsta",
    "bodies.status.not_configured": "ej konfigurerad",
    "bodies.status.unobserved": "Inte observerad",
    "bodies.title": "Kroppar",
    "events.actions.export": "Exportera JSONL",
    "events.actions.pause": "Pausa ström",
    "events.description": "Tekniska händelser från agent, modell, research och framtida robotnoder.",
    "events.detail.close": "Stäng händelsedetaljer",
    "events.detail.copy": "Kopiera",
    "events.detail.eyebrow": "Händelsedata",
    "events.detail.raw_json": "Rå JSON",
    "events.detail.title": "Händelsedetaljer",
    "events.eyebrow": "Observationsström med endast tillägg",
    "events.filters.all": "Alla",
    "events.filters.aria_label": "Loggfilter",
    "events.filters.plane.agent": "Agent",
    "events.filters.plane.dashboard": "Dashboard",
    "events.filters.plane.label": "Plan",
    "events.filters.plane.model": "Modell",
    "events.filters.plane.perception": "Perception",
    "events.filters.plane.policy": "Policy",
    "events.filters.plane.research": "Research",
    "events.filters.plane.supervisor": "Övervakare",
    "events.filters.plane.transport": "Transport",
    "events.filters.search.label": "Sök i sammanfattning",
    "events.filters.search.placeholder": "ID eller exakt loggtext",
    "events.filters.severity.critical": "Kritisk",
    "events.filters.severity.debug": "Felsökning",
    "events.filters.severity.error": "Fel",
    "events.filters.severity.info": "Info",
    "events.filters.severity.label": "Nivå",
    "events.filters.severity.warning": "Varning",
    "events.stream.connecting": "Ansluter…",
    "events.table.caption": "Teknisk händelselogg",
    "events.table.details": "Detaljer",
    "events.table.empty": "Väntar på den första tekniska händelsen.",
    "events.table.event": "Händelse",
    "events.table.plane": "Plan",
    "events.table.severity": "Nivå",
    "events.table.summary": "Sammanfattning",
    "events.table.time": "Tid",
    "events.title": "Händelser",
    "experiments.description": "Verifierade körningar, konfigurationer och evidens. Den här vyn startar aldrig hårdvara.",
    "experiments.empty.body": "När episoder senare sparas här kommer git-SHA, konfiguration, evidens och slutsats att följa med.",
    "experiments.empty.eyebrow": "Ingen API-historik",
    "experiments.empty.title": "Experimentregistret är tomt.",
    "experiments.eyebrow": "Reproducerbar loggbok",
    "experiments.history": "Historik",
    "experiments.title": "Experiment",
    "footer.bodies": "Kroppar",
    "footer.events": "Händelser",
    "footer.experiments": "Experiment",
    "footer.mobile_nav.aria_label": "Mobil navigation",
    "footer.settings": "Inställningar",
    "footer.workbench": "Arbetsbänk",
    "inspector.activity.empty_body": "Typade beslut och verktygsanrop visas här.",
    "inspector.activity.empty_title": "Ingen episod ännu",
    "inspector.activity.eyebrow": "Senaste episod",
    "inspector.activity.title": "Agentaktivitet",
    "inspector.activity.waiting": "Väntar",
    "inspector.aria_label": "Agentinspektör",
    "inspector.close": "Stäng inspektör",
    "inspector.context.body": "Endast konversationens synliga turer och versionsmärkta fakta visas. Dolda resonemang loggas inte.",
    "inspector.context.context_version": "Kontextversion",
    "inspector.context.conversation_id": "Konversations-ID",
    "inspector.context.eyebrow": "Synligt minne",
    "inspector.context.not_created": "Inte skapad",
    "inspector.context.title": "Kontext",
    "inspector.context.turn_count": "Antal turer",
    "inspector.evidence.empty": "Ingen extern evidens har hämtats.",
    "inspector.evidence.eyebrow": "Passiv kunskap",
    "inspector.evidence.help": "Researchresultat visas med leverantör, giltighet och hash.",
    "inspector.evidence.title": "Evidens",
    "inspector.metrics.context_version": "Kontextversion",
    "inspector.metrics.planner_turns": "Planerarsteg",
    "inspector.metrics.replans": "Omplaneringar",
    "inspector.metrics.tool_calls": "Verktygsanrop",
    "inspector.tabs.activity": "Aktivitet",
    "inspector.tabs.aria_label": "Detaljvy",
    "inspector.tabs.context": "Kontext",
    "inspector.tabs.evidence": "Evidens",
    "nav.aria_label": "Huvudnavigation",
    "nav.bodies.subtitle": "Noder och status",
    "nav.bodies.title": "Kroppar",
    "nav.events.subtitle": "Teknisk logg",
    "nav.events.title": "Händelser",
    "nav.experiments.subtitle": "Evidens och historik",
    "nav.experiments.title": "Experiment",
    "nav.group.lab": "Labbet",
    "nav.safety.body": "Motorstyrning, SSH och TTS är avstängda i den här versionen.",
    "nav.safety.title": "Fysisk styrning avstängd",
    "nav.settings.subtitle": "Runtime och budgetar",
    "nav.settings.title": "Inställningar",
    "nav.workbench.subtitle": "Chatt och aktivitet",
    "nav.workbench.title": "Arbetsbänk",
    "settings.actions.clean": "Inga osparade ändringar.",
    "settings.actions.reset": "Återställ",
    "settings.actions.save": "Spara lokalt",
    "settings.budgets.default_mode": "Standardläge",
    "settings.budgets.description": "Hårda tak för varje lokal episod.",
    "settings.budgets.evidence_ttl": "Evidens-TTL, ms",
    "settings.budgets.log_debug": "Felsökning",
    "settings.budgets.log_error": "Fel",
    "settings.budgets.log_info": "Info",
    "settings.budgets.log_level": "Loggnivå",
    "settings.budgets.log_warning": "Varning",
    "settings.budgets.mode_conversation": "Lokalt samtal",
    "settings.budgets.mode_research": "Research",
    "settings.budgets.planner_turns": "Planerarsteg",
    "settings.budgets.replans": "Omplaneringar",
    "settings.budgets.title": "Agentbudgetar",
    "settings.budgets.tool_calls": "Verktygsanrop",
    "settings.budgets.tool_request_ttl": "Verktygsbegärans TTL, ms",
    "settings.budgets.total_time": "Total tid, ms",
    "settings.budgets.weather_skew": "Maximal väderskevhet, ms",
    "settings.description": "Modell, agentbudgetar och loggning. Fysisk armering hör inte hemma här.",
    "settings.eyebrow": "Lokal konfiguration",
    "settings.revision.empty": "Revision —",
    "settings.runtime.description": "Endast en lokal loopback-slutpunkt accepteras.",
    "settings.runtime.endpoint": "Slutpunkt",
    "settings.runtime.endpoint_help": "Runtimevärde · ändras utanför dashboarden",
    "settings.runtime.model": "Modell",
    "settings.runtime.planner_timeout": "Planerarens tidsgräns, ms",
    "settings.runtime.probe": "Testa anslutning",
    "settings.runtime.probe_idle": "Ingen anslutningskontroll körd ännu.",
    "settings.safety.banner_body": "Ingen exekveringsprofil är installerad. Fysisk armering blir senare ett separat, kortlivat och granskat arbetsflöde.",
    "settings.safety.banner_title": "Rörelse är inte aktiverad i denna version.",
    "settings.safety.description": "Informationsyta, inte en kontroll.",
    "settings.safety.locked": "Låst",
    "settings.safety.not_exposed": "Ej exponerat",
    "settings.safety.off": "Av",
    "settings.safety.physical_control": "Fysisk styrning",
    "settings.safety.read_only": "Skrivskyddat",
    "settings.safety.research": "Research",
    "settings.safety.title": "Fysisk säkerhet",
    "settings.title": "Inställningar",
    "settings.tools.available": "Tillgängligt",
    "settings.tools.description": "Exakt tillåtelselista. Verktyg får endast producera passiv evidens.",
    "settings.tools.generic_fetch": "Generisk webbhämtning",
    "settings.tools.generic_fetch_help": "Väntar på DNS-pinning och SSRF-grind",
    "settings.tools.not_installed": "Ej installerad",
    "settings.tools.title": "Verktyg",
    "settings.tools.weather_origin": "Open-Meteo · fast HTTPS-ursprung",
    "shell.brand.name": "Robot LLM Lab",
    "shell.brand.tagline": "Lokal agent- och robotarbetsbänk",
    "shell.document_title": "Robot LLM Lab",
    "shell.local.label": "Lokalt",
    "shell.local.title": "All applikationslogik körs på denna Mac",
    "shell.runtime.aria_label": "Systemstatus",
    "shell.runtime.model.checking": "kontrollerar…",
    "shell.runtime.motion.locked": "låst",
    "shell.runtime.motion.name": "Rörelse",
    "shell.runtime.research.name": "Aktuell info",
    "shell.runtime.research.ready": "väder redo",
    "shell.runtime.robot.unobserved": "inte observerad",
    "shell.skip_to_content": "Hoppa till innehållet",
    "shell.value.unavailable": "—",
    "workbench.actions.activity": "Aktivitet",
    "workbench.actions.new_conversation": "Ny konversation",
    "workbench.composer.message_label": "Meddelande till Gemma",
    "workbench.composer.placeholder": "Prata med Gemma eller be den undersöka något…",
    "workbench.composer.send": "Skicka",
    "workbench.composer.starting": "Startar den lokala arbetsbänken…",
    "workbench.history.aria_label": "Konversationshistorik",
    "workbench.mode.capability": "Ingen robotstyrning",
    "workbench.mode.conversation": "Lokalt samtal · kan kolla upp saker vid behov",
    "workbench.mode.label": "Körläge",
    "workbench.mode.research": "Aktuell information · färska källor krävs",
    "workbench.session.eyebrow": "Aktiv session",
    "workbench.starter.aria_label": "Exempelfrågor",
    "workbench.starter.capabilities.label": "Vad kan du göra just nu?",
    "workbench.starter.capabilities.prompt": "Förklara vad du kan göra i det här läget.",
    "workbench.starter.weather.label": "Behöver jag paraply i Stockholm?",
    "workbench.starter.weather.prompt": "Behöver jag paraply i Stockholm just nu?",
    "workbench.subtitle": "Lokal Gemma · robotstyrning avstängd",
    "workbench.title": "Arbetsbänk",
    "workbench.welcome.body": "Jag kör lokalt på din Mac. Vi kan prata eller kolla upp aktuell information, som väder. Jag styr inte robotkroppen från den här chatten ännu.",
    "workbench.welcome.eyebrow": "Redo när du är",
    "workbench.welcome.mascot_alt": "Robot LLM Labs lätt griniga modulrobot vinkar",
    "workbench.welcome.title": "Hallå! Vad ska vi hitta på?",
    "state.unknown": "okänd",
    "state.online": "ansluten",
    "state.offline": "offline",
    "state.unobserved": "inte observerad",
    "state.configured": "konfigurerad",
    "state.active": "aktiv",
    "state.inactive": "inaktiv",
    "state.queued": "köad",
    "state.running": "arbetar",
    "state.answered": "besvarad",
    "state.clarification_required": "behöver förtydligande",
    "state.failed": "misslyckad",
    "state.verified": "verifierad",
    "state.waiting": "väntar",
    "runtime.model_not_loaded": "konfigurerad modell ej laddad",
    "runtime.connected": "ansluten",
    "runtime.no_model": "ingen modell",
    "runtime.checking": "Kontrollerar den lokala runtime-processen…",
    "runtime.dashboard_offline": "dashboard offline",
    "runtime.probe_failed": "Anslutningskontrollen misslyckades.",
    "capability.locked": "låst",
    "capability.contract_breach": "kontraktsbrott",
    "capability.rejected": "capability avvisad",
    "capability.unavailable": "inte tillgänglig",
    "capability.weather_ready": "väder redo",
    "capability.chat_ready": "Redo att chatta · robotstyrning är avstängd",
    "capability.model_not_ready": "LM Studio eller den konfigurerade modellen är inte redo",
    "capability.chat_unavailable": "Chatt är inte tillgänglig",
    "capability.read_only_violation": "Servern bröt mot dashboardens read-only-kontrakt. Mutationer har stängts.",
    "registry.unnamed_node": "Namnlös nod",
    "registry.robot": "robot",
    "registry.node": "nod",
    "registry.host_and_providers": "Värd och leverantörer",
    "registry.names.composite_lab_robot": "Sammansatt labbrobot",
    "registry.names.front_camera": "Frontkamera",
    "registry.names.microphone_array": "Mikrofonmatris",
    "registry.names.vision_node": "Visionsnod",
    "registry.names.audio_node": "Ljudnod",
    "registry.names.mac_host": "Mac-värd",
    "registry.future_sources": "Kameror och mikrofoner",
    "registry.future_sources_note": "framtida perceptionskällor",
    "registry.not_configured": "ej konfigurerade",
    "registry.field.state": "Tillstånd",
    "registry.field.controller_id": "Styrenhets-ID",
    "registry.field.instance_id": "Instans-ID",
    "registry.field.last_observed": "Senast observerad",
    "registry.field.status_reason": "Statusorsak",
    "registry.field.physical_capabilities": "Fysiska funktioner",
    "registry.physical_rejected": "Avvisade",
    "registry.physical_locked": "Låsta",
    "registry.not_observed": "Inte observerad",
    "experiments.missing_id": "EXP-—",
    "experiments.untitled": "Namnlöst experiment",
    "experiments.no_summary": "Ingen sammanfattning.",
    "experiments.curated.dynamic_ir.title": "Dynamisk IR-evidens",
    "experiments.curated.dynamic_ir.summary": "277 motorstilla prover verifierar den provisoriska närhetsgrinden.",
    "experiments.curated.weather_tool.title": "Gemma väljer weather.current",
    "experiments.curated.weather_tool.summary": "Tvåstegs plan–tool–answer-loop med bunden evidens.",
    "experiments.curated.ev3_preflight.title": "Fysisk foreground-preflight",
    "experiments.curated.ev3_preflight.summary": "Väntar på batterier; inga motorstartkommandon skickas.",
    "settings.unsaved": "Osparade lokala ändringar.",
    "settings.no_unsaved": "Inga osparade ändringar.",
    "settings.revision": ({ revision }) => `Revision ${revision}`,
    "settings.saving": "Sparar och validerar…",
    "settings.saved": "Inställningarna sparades lokalt.",
    "settings.save_failed": "Inställningarna kunde inte sparas.",
    "chat.author.user": "Du",
    "chat.author.assistant": "Gemma",
    "chat.author.system": "System",
    "chat.history_evidence_failed": "Historisk evidens kunde inte hämtas.",
    "chat.working": "Arbetar",
    "chat.episode_aborted": ({ code }) => `Episoden avbröts: ${code}`,
    "chat.context.conversation_id": "Konversations-ID",
    "chat.context.version": "Kontextversion",
    "chat.context.mode": "Kontextläge",
    "chat.context.turn_count": "Antal turer",
    "chat.context.not_created": "Inte skapad",
    "chat.activity.waiting": "Väntar",
    "chat.activity.no_episode": "Ingen episod ännu",
    "chat.activity.note": "Typade beslut och verktygsanrop visas här.",
    "chat.evidence.empty": "Ingen extern evidens har hämtats.",
    "chat.evidence.empty_note": "Researchresultat visas med hostmyntade evidence-ID:n.",
    "chat.evidence.verified_fallback": "Verifierad citation från den skrivskyddade researchloopen.",
    "chat.evidence.read_only": "skrivskyddad",
    "chat.evidence.validity": "Giltigheten verifieras av den lokala värden",
    "chat.answer_ready": "Svar verifierat · redo för nästa tur",
    "chat.clarification_needed": "Ett förtydligande behövs",
    "chat.episode_stopped": ({ code }) => `Episoden stoppades · ${code}`,
    "chat.announcer.answer": ({ text }) => `Gemma svarade: ${text}`,
    "chat.announcer.clarification": ({ text }) => `Gemma behöver ett förtydligande: ${text}`,
    "chat.announcer.stopped": "Agentepisoden stoppades utan svar.",
    "chat.turn_progress": ({ state, id }) => `${state} · ${id}`,
    "chat.conversation_version": ({ mode, version }) => `${mode} · version ${version}`,
    "chat.wait_for_terminal": "Vänta tills den pågående episoden är terminal.",
    "chat.created": "Ny lokal konversation skapad.",
    "chat.create_failed": "Konversationen kunde inte skapas.",
    "chat.poll_failed": "Tillfälligt anslutningsfel. Försöker hämta turnstatus igen…",
    "chat.poll_connection_unknown": "Kontakten med turnstatus är bruten. Agenten kan fortfarande arbeta; inmatningen förblir låst medan vi försöker återansluta.",
    "chat.poll_recovered": "Kontakten med agentkörningen är återställd.",
    "chat.episode_in_progress": "En episod pågår redan.",
    "chat.send_failed": "Meddelandet kunde inte skickas.",
    "events.detail_fallback": "Händelsedetaljer",
    "events.field.event_id": "Händelse-ID",
    "events.field.sequence": "Sekvens",
    "events.field.time": "Tid",
    "events.field.level": "Nivå",
    "events.field.category": "Kategori",
    "events.field.source_id": "Käll-ID",
    "events.field.robot_id": "Robot-ID",
    "events.field.node_id": "Nod-ID",
    "events.field.conversation_id": "Konversations-ID",
    "events.field.turn_id": "Tur-ID",
    "events.field.tool_call_id": "Verktygsanrops-ID",
    "events.field.request_id": "Begärans-ID",
    "events.empty": "Väntar på den första tekniska händelsen.",
    "events.no_match": "Inga händelser matchar filtret.",
    "events.show": ({ type }) => `Visa ${type}`,
    "events.event_fallback": "händelse",
    "events.gap": ({ count }) => `Loggströmmen saknar händelser. Totalt antal bortfallna händelser: ${count}.`,
    "events.live": ({ sequence }) => `Live · sekvens ${sequence}`,
    "events.offline": "Loggserver offline",
    "events.resume": "Återuppta ström",
    "events.pause": "Pausa ström",
    "events.paused": "Pausad lokalt",
    "events.reconnecting": "Återansluter…",
    "events.nothing_to_export": "Det finns inga lokala events att exportera.",
    "events.exported": "Den redigerade lokala eventbufferten exporterades.",
    "events.json_copied": "Händelsens JSON kopierades.",
    "events.json_copy_failed": "JSON kunde inte kopieras.",
    "mode.research_note": "Aktuella väderkällor krävs · ingen robotstyrning",
    "mode.conversation_note": "Kan kolla upp väder vid behov · ingen robotstyrning",
    "server.unreachable": "Dashboardservern kunde inte nås.",
    "server.start_failed": "Dashboardservern kunde inte startas.",
    "errors.generic": "Något gick fel i den lokala dashboarden.",
    "errors.dashboard_session_missing": "Dashboardens sessionsnyckel saknas.",
    "errors.invalid_server_json": "Servern returnerade ogiltig JSON.",
    "errors.request_timeout": "Den lokala servern svarade inte inom tidsgränsen.",
    "errors.network_error": "Den lokala servern kunde inte nås.",
    "errors.origin_rejected": "Anropet avvisades eftersom sidans origin inte är tillåten.",
    "errors.session_token_rejected": "Dashboardens sessionsnyckel avvisades.",
    "errors.invalid_request": "Servern avvisade den ogiltiga förfrågan.",
    "errors.conversation_not_found": "Konversationen finns inte längre på servern.",
    "errors.turn_not_found": "Agentturen finns inte längre på servern.",
    "errors.settings_revision_conflict": "Inställningarna ändrades av någon annan. Läs in den senaste revisionen och försök igen.",
    "errors.conversation_version_conflict": "Konversationen ändrades innan turen kunde starta. Läs in den senaste versionen och försök igen.",
    "errors.duplicate_client_request": "Samma klientförfrågan har redan behandlats.",
    "errors.idempotency_conflict": "Idempotensnyckeln används redan för en annan förfrågan.",
    "errors.chat_queue_full": "Agentens lokala arbetskö är full. Försök igen när en pågående episod är klar.",
    "errors.service_stopping": "Den lokala agenttjänsten håller på att stängas.",
    "errors.conversation_turn_active": "Konversationen har redan en pågående agenttur.",
    "errors.invalid_response_locale": "Servern avvisade det valda svarsspråket.",
    "errors.invalid_chat_mode": "Servern avvisade det valda körläget.",
    "errors.runtime_unavailable": "Den lokala modellruntime-processen är inte tillgänglig.",
    "errors.model_not_ready": "Den konfigurerade modellen är inte redo.",
  });

  const en = Object.freeze({
    "common.missing": "—",
    "common.unknown": "unknown",
    "common.none": "none",
    "locale.swedish": "Svenska",
    "locale.english": "English",
    "locale.label": "Language",
    "locale.selector.aria_label": "Choose language",
    "bodies.controller.controller_id": "Controller ID",
    "bodies.controller.eyebrow": "Controller details",
    "bodies.controller.heartbeat": "Heartbeat",
    "bodies.controller.instance_id": "Instance ID",
    "bodies.controller.physical_capabilities": "Physical capabilities",
    "bodies.controller.state": "State",
    "bodies.controller.state_version": "State version",
    "bodies.description": "Controllers execute. Cameras and microphones observe. They are different kinds of nodes.",
    "bodies.eyebrow": "Distributed robot body",
    "bodies.logical.eyebrow": "Logical body",
    "bodies.nodes.controller_transport": "controller · USB/SSH",
    "bodies.nodes.front_camera": "Front camera",
    "bodies.nodes.future_perception": "future perception source",
    "bodies.nodes.microphone_array": "Microphone array",
    "bodies.nodes.robot_waiting": "robot_id waiting for registry",
    "bodies.read_only": "Read-only inventory",
    "bodies.safety.body": "Seeing a controller in the registry never automatically authorizes motion.",
    "bodies.safety.title": "No execution path",
    "bodies.status.configured": "configured",
    "bodies.status.locked_plural": "Locked",
    "bodies.status.not_configured": "not configured",
    "bodies.status.unobserved": "Not observed",
    "bodies.title": "Bodies",
    "events.actions.export": "Export JSONL",
    "events.actions.pause": "Pause stream",
    "events.description": "Technical events from the agent, model, research, and future robot nodes.",
    "events.detail.close": "Close event details",
    "events.detail.copy": "Copy",
    "events.detail.eyebrow": "Event data",
    "events.detail.raw_json": "Raw JSON",
    "events.detail.title": "Event details",
    "events.eyebrow": "Append-only observation stream",
    "events.filters.all": "All",
    "events.filters.aria_label": "Log filters",
    "events.filters.plane.agent": "Agent",
    "events.filters.plane.dashboard": "Dashboard",
    "events.filters.plane.label": "Plane",
    "events.filters.plane.model": "Model",
    "events.filters.plane.perception": "Perception",
    "events.filters.plane.policy": "Policy",
    "events.filters.plane.research": "Research",
    "events.filters.plane.supervisor": "Supervisor",
    "events.filters.plane.transport": "Transport",
    "events.filters.search.label": "Search summaries",
    "events.filters.search.placeholder": "ID or exact log text",
    "events.filters.severity.critical": "Critical",
    "events.filters.severity.debug": "Debug",
    "events.filters.severity.error": "Error",
    "events.filters.severity.info": "Info",
    "events.filters.severity.label": "Level",
    "events.filters.severity.warning": "Warning",
    "events.stream.connecting": "Connecting…",
    "events.table.caption": "Technical event log",
    "events.table.details": "Details",
    "events.table.empty": "Waiting for the first technical event.",
    "events.table.event": "Event",
    "events.table.plane": "Plane",
    "events.table.severity": "Level",
    "events.table.summary": "Summary",
    "events.table.time": "Time",
    "events.title": "Events",
    "experiments.description": "Verified runs, configurations, and evidence. This view never starts hardware.",
    "experiments.empty.body": "When episodes are saved here later, their git SHA, configuration, evidence, and conclusion will be included.",
    "experiments.empty.eyebrow": "No API history",
    "experiments.empty.title": "The experiment registry is empty.",
    "experiments.eyebrow": "Reproducible logbook",
    "experiments.history": "History",
    "experiments.title": "Experiments",
    "footer.bodies": "Bodies",
    "footer.events": "Events",
    "footer.experiments": "Experiments",
    "footer.mobile_nav.aria_label": "Mobile navigation",
    "footer.settings": "Settings",
    "footer.workbench": "Workbench",
    "inspector.activity.empty_body": "Typed decisions and tool calls are shown here.",
    "inspector.activity.empty_title": "No episode yet",
    "inspector.activity.eyebrow": "Latest episode",
    "inspector.activity.title": "Agent activity",
    "inspector.activity.waiting": "Waiting",
    "inspector.aria_label": "Agent inspector",
    "inspector.close": "Close inspector",
    "inspector.context.body": "Only visible conversation turns and versioned facts are shown. Hidden reasoning is not logged.",
    "inspector.context.context_version": "Context version",
    "inspector.context.conversation_id": "Conversation ID",
    "inspector.context.eyebrow": "Visible memory",
    "inspector.context.not_created": "Not created",
    "inspector.context.title": "Context",
    "inspector.context.turn_count": "Turn count",
    "inspector.evidence.empty": "No external evidence has been retrieved.",
    "inspector.evidence.eyebrow": "Passive knowledge",
    "inspector.evidence.help": "Research results are shown with provider, validity, and hash.",
    "inspector.evidence.title": "Evidence",
    "inspector.metrics.context_version": "Context version",
    "inspector.metrics.planner_turns": "Planner turns",
    "inspector.metrics.replans": "Replans",
    "inspector.metrics.tool_calls": "Tool calls",
    "inspector.tabs.activity": "Activity",
    "inspector.tabs.aria_label": "Detail view",
    "inspector.tabs.context": "Context",
    "inspector.tabs.evidence": "Evidence",
    "nav.aria_label": "Main navigation",
    "nav.bodies.subtitle": "Nodes and status",
    "nav.bodies.title": "Bodies",
    "nav.events.subtitle": "Technical log",
    "nav.events.title": "Events",
    "nav.experiments.subtitle": "Evidence and history",
    "nav.experiments.title": "Experiments",
    "nav.group.lab": "Lab",
    "nav.safety.body": "Motor control, SSH, and TTS are disabled in this version.",
    "nav.safety.title": "Physical control disabled",
    "nav.settings.subtitle": "Runtime and budgets",
    "nav.settings.title": "Settings",
    "nav.workbench.subtitle": "Chat and activity",
    "nav.workbench.title": "Workbench",
    "settings.actions.clean": "No unsaved changes.",
    "settings.actions.reset": "Reset",
    "settings.actions.save": "Save locally",
    "settings.budgets.default_mode": "Default mode",
    "settings.budgets.description": "Hard limits for each local episode.",
    "settings.budgets.evidence_ttl": "Evidence TTL, ms",
    "settings.budgets.log_debug": "Debug",
    "settings.budgets.log_error": "Error",
    "settings.budgets.log_info": "Info",
    "settings.budgets.log_level": "Log level",
    "settings.budgets.log_warning": "Warning",
    "settings.budgets.mode_conversation": "Local conversation",
    "settings.budgets.mode_research": "Research",
    "settings.budgets.planner_turns": "Planner turns",
    "settings.budgets.replans": "Replans",
    "settings.budgets.title": "Agent budgets",
    "settings.budgets.tool_calls": "Tool calls",
    "settings.budgets.tool_request_ttl": "Tool request TTL, ms",
    "settings.budgets.total_time": "Total time, ms",
    "settings.budgets.weather_skew": "Maximum weather skew, ms",
    "settings.description": "Model, agent budgets, and logging. Physical arming is intentionally handled elsewhere.",
    "settings.eyebrow": "Local configuration",
    "settings.revision.empty": "Revision —",
    "settings.runtime.description": "Only a local loopback endpoint is accepted.",
    "settings.runtime.endpoint": "Endpoint",
    "settings.runtime.endpoint_help": "Runtime value · changed outside the dashboard",
    "settings.runtime.model": "Model",
    "settings.runtime.planner_timeout": "Planner timeout, ms",
    "settings.runtime.probe": "Test connection",
    "settings.runtime.probe_idle": "No connection check has run yet.",
    "settings.safety.banner_body": "No execution profile is installed. Physical arming will be introduced later as a separate, time-limited, and auditable workflow.",
    "settings.safety.banner_title": "Motion is not enabled in this version.",
    "settings.safety.description": "Status information only — not a control.",
    "settings.safety.locked": "Locked",
    "settings.safety.not_exposed": "Not exposed",
    "settings.safety.off": "Off",
    "settings.safety.physical_control": "Physical control",
    "settings.safety.read_only": "Read-only",
    "settings.safety.research": "Research",
    "settings.safety.title": "Physical safety",
    "settings.title": "Settings",
    "settings.tools.available": "Available",
    "settings.tools.description": "Exact allowlist. Tools may only produce passive evidence.",
    "settings.tools.generic_fetch": "Generic web fetch",
    "settings.tools.generic_fetch_help": "Waiting for DNS pinning and an SSRF gate",
    "settings.tools.not_installed": "Not installed",
    "settings.tools.title": "Tools",
    "settings.tools.weather_origin": "Open-Meteo · fixed HTTPS origin",
    "shell.brand.name": "Robot LLM Lab",
    "shell.brand.tagline": "Local agent and robot workbench",
    "shell.document_title": "Robot LLM Lab",
    "shell.local.label": "Local",
    "shell.local.title": "All application logic runs on this Mac",
    "shell.runtime.aria_label": "System status",
    "shell.runtime.model.checking": "checking…",
    "shell.runtime.motion.locked": "locked",
    "shell.runtime.motion.name": "Motion",
    "shell.runtime.research.name": "Current info",
    "shell.runtime.research.ready": "weather ready",
    "shell.runtime.robot.unobserved": "not observed",
    "shell.skip_to_content": "Skip to content",
    "shell.value.unavailable": "—",
    "workbench.actions.activity": "Activity",
    "workbench.actions.new_conversation": "New conversation",
    "workbench.composer.message_label": "Message to Gemma",
    "workbench.composer.placeholder": "Talk to Gemma or ask it to investigate something…",
    "workbench.composer.send": "Send",
    "workbench.composer.starting": "Starting the local workbench…",
    "workbench.history.aria_label": "Conversation history",
    "workbench.mode.capability": "No robot control",
    "workbench.mode.conversation": "Local conversation · can look things up when needed",
    "workbench.mode.label": "Run mode",
    "workbench.mode.research": "Current information · fresh sources required",
    "workbench.session.eyebrow": "Active session",
    "workbench.starter.aria_label": "Example questions",
    "workbench.starter.capabilities.label": "What can you do right now?",
    "workbench.starter.capabilities.prompt": "Explain what you can do in this mode.",
    "workbench.starter.weather.label": "Do I need an umbrella in Stockholm?",
    "workbench.starter.weather.prompt": "Do I need an umbrella in Stockholm right now?",
    "workbench.subtitle": "Local Gemma · robot control disabled",
    "workbench.title": "Workbench",
    "workbench.welcome.body": "I run locally on your Mac. We can talk or look up current information, such as weather. I do not control the robot body from this chat yet.",
    "workbench.welcome.eyebrow": "Ready when you are",
    "workbench.welcome.mascot_alt": "Robot LLM Lab’s mildly grumpy modular robot waves",
    "workbench.welcome.title": "Hello! What shall we do?",
    "state.unknown": "unknown",
    "state.online": "connected",
    "state.offline": "offline",
    "state.unobserved": "not observed",
    "state.configured": "configured",
    "state.active": "active",
    "state.inactive": "inactive",
    "state.queued": "queued",
    "state.running": "working",
    "state.answered": "answered",
    "state.clarification_required": "needs clarification",
    "state.failed": "failed",
    "state.verified": "verified",
    "state.waiting": "waiting",
    "runtime.model_not_loaded": "configured model not loaded",
    "runtime.connected": "connected",
    "runtime.no_model": "no model",
    "runtime.checking": "Checking the local runtime process…",
    "runtime.dashboard_offline": "dashboard offline",
    "runtime.probe_failed": "Connection check failed.",
    "capability.locked": "locked",
    "capability.contract_breach": "contract breach",
    "capability.rejected": "capability rejected",
    "capability.unavailable": "unavailable",
    "capability.weather_ready": "weather ready",
    "capability.chat_ready": "Ready to chat · robot control is disabled",
    "capability.model_not_ready": "LM Studio or the configured model is not ready",
    "capability.chat_unavailable": "Chat is unavailable",
    "capability.read_only_violation": "The server breached the dashboard’s read-only contract. Mutations have been disabled.",
    "registry.unnamed_node": "Unnamed node",
    "registry.robot": "robot",
    "registry.node": "node",
    "registry.host_and_providers": "Host & providers",
    "registry.names.composite_lab_robot": "Composite Lab Robot",
    "registry.names.front_camera": "Front Camera",
    "registry.names.microphone_array": "Microphone Array",
    "registry.names.vision_node": "Vision Node",
    "registry.names.audio_node": "Audio Node",
    "registry.names.mac_host": "Mac Host",
    "registry.future_sources": "Cameras and microphones",
    "registry.future_sources_note": "future perception sources",
    "registry.not_configured": "not configured",
    "registry.field.state": "State",
    "registry.field.controller_id": "Controller ID",
    "registry.field.instance_id": "Instance ID",
    "registry.field.last_observed": "Last observed",
    "registry.field.status_reason": "Status reason",
    "registry.field.physical_capabilities": "Physical capabilities",
    "registry.physical_rejected": "Rejected",
    "registry.physical_locked": "Locked",
    "registry.not_observed": "Not observed",
    "experiments.missing_id": "EXP-—",
    "experiments.untitled": "Untitled experiment",
    "experiments.no_summary": "No summary.",
    "experiments.curated.dynamic_ir.title": "Dynamic IR evidence",
    "experiments.curated.dynamic_ir.summary": "277 stationary-motor samples verify the provisional proximity gate.",
    "experiments.curated.weather_tool.title": "Gemma selects weather.current",
    "experiments.curated.weather_tool.summary": "Two-stage plan–tool–answer loop with bound evidence.",
    "experiments.curated.ev3_preflight.title": "Physical foreground preflight",
    "experiments.curated.ev3_preflight.summary": "Waiting for batteries; no motor-start commands are sent.",
    "settings.unsaved": "Unsaved local changes.",
    "settings.no_unsaved": "No unsaved changes.",
    "settings.revision": ({ revision }) => `Revision ${revision}`,
    "settings.saving": "Saving and validating…",
    "settings.saved": "Settings saved locally.",
    "settings.save_failed": "Settings could not be saved.",
    "chat.author.user": "You",
    "chat.author.assistant": "Gemma",
    "chat.author.system": "System",
    "chat.history_evidence_failed": "Historical evidence could not be loaded.",
    "chat.working": "Working",
    "chat.episode_aborted": ({ code }) => `Episode aborted: ${code}`,
    "chat.context.conversation_id": "Conversation ID",
    "chat.context.version": "Context version",
    "chat.context.mode": "Context mode",
    "chat.context.turn_count": "Turn count",
    "chat.context.not_created": "Not created",
    "chat.activity.waiting": "Waiting",
    "chat.activity.no_episode": "No episode yet",
    "chat.activity.note": "Typed decisions and tool calls are shown here.",
    "chat.evidence.empty": "No external evidence has been retrieved.",
    "chat.evidence.empty_note": "Research results appear with host-minted evidence IDs.",
    "chat.evidence.verified_fallback": "Verified citation from the read-only research loop.",
    "chat.evidence.read_only": "read-only",
    "chat.evidence.validity": "Freshness is verified by the local host",
    "chat.answer_ready": "Answer verified · ready for the next turn",
    "chat.clarification_needed": "Clarification is needed",
    "chat.episode_stopped": ({ code }) => `Episode stopped · ${code}`,
    "chat.announcer.answer": ({ text }) => `Gemma answered: ${text}`,
    "chat.announcer.clarification": ({ text }) => `Gemma needs clarification: ${text}`,
    "chat.announcer.stopped": "The agent episode stopped without an answer.",
    "chat.turn_progress": ({ state, id }) => `${state} · ${id}`,
    "chat.conversation_version": ({ mode, version }) => `${mode} · version ${version}`,
    "chat.wait_for_terminal": "Wait until the current episode reaches a terminal state.",
    "chat.created": "New local conversation created.",
    "chat.create_failed": "The conversation could not be created.",
    "chat.poll_failed": "Temporary connection error. Retrying the turn status…",
    "chat.poll_connection_unknown": "The turn-status connection is unavailable. The agent may still be working; input stays locked while we reconnect.",
    "chat.poll_recovered": "The connection to the agent run has recovered.",
    "chat.episode_in_progress": "An episode is already in progress.",
    "chat.send_failed": "The message could not be sent.",
    "events.detail_fallback": "Event details",
    "events.field.event_id": "Event ID",
    "events.field.sequence": "Sequence",
    "events.field.time": "Time",
    "events.field.level": "Level",
    "events.field.category": "Category",
    "events.field.source_id": "Source ID",
    "events.field.robot_id": "Robot ID",
    "events.field.node_id": "Node ID",
    "events.field.conversation_id": "Conversation ID",
    "events.field.turn_id": "Turn ID",
    "events.field.tool_call_id": "Tool call ID",
    "events.field.request_id": "Request ID",
    "events.empty": "Waiting for the first technical event.",
    "events.no_match": "No events match the filter.",
    "events.show": ({ type }) => `Show ${type}`,
    "events.event_fallback": "event",
    "events.gap": ({ count }) => `Some log events are missing. Total events dropped: ${count}.`,
    "events.live": ({ sequence }) => `Live · sequence ${sequence}`,
    "events.offline": "Log server offline",
    "events.resume": "Resume stream",
    "events.pause": "Pause stream",
    "events.paused": "Paused locally",
    "events.reconnecting": "Reconnecting…",
    "events.nothing_to_export": "There are no local events to export.",
    "events.exported": "The redacted local event buffer was exported.",
    "events.json_copied": "The event JSON was copied.",
    "events.json_copy_failed": "JSON could not be copied.",
    "mode.research_note": "Current weather sources required · no robot control",
    "mode.conversation_note": "Can look up weather when needed · no robot control",
    "server.unreachable": "The dashboard server could not be reached.",
    "server.start_failed": "The dashboard server could not be started.",
    "errors.generic": "Something went wrong in the local dashboard.",
    "errors.dashboard_session_missing": "The dashboard session key is missing.",
    "errors.invalid_server_json": "The server returned invalid JSON.",
    "errors.request_timeout": "The local server did not respond before the timeout.",
    "errors.network_error": "The local server could not be reached.",
    "errors.origin_rejected": "The request was rejected because the page origin is not allowed.",
    "errors.session_token_rejected": "The dashboard session key was rejected.",
    "errors.invalid_request": "The server rejected the invalid request.",
    "errors.conversation_not_found": "The conversation no longer exists on the server.",
    "errors.turn_not_found": "The agent turn no longer exists on the server.",
    "errors.settings_revision_conflict": "The settings were changed elsewhere. Load the latest revision and try again.",
    "errors.conversation_version_conflict": "The conversation changed before the turn could start. Load the latest version and try again.",
    "errors.duplicate_client_request": "The same client request has already been handled.",
    "errors.idempotency_conflict": "The idempotency key is already being used for a different request.",
    "errors.chat_queue_full": "The agent’s local work queue is full. Try again when an active episode has finished.",
    "errors.service_stopping": "The local agent service is shutting down.",
    "errors.conversation_turn_active": "The conversation already has an active agent turn.",
    "errors.invalid_response_locale": "The server rejected the selected response language.",
    "errors.invalid_chat_mode": "The server rejected the selected run mode.",
    "errors.runtime_unavailable": "The local model runtime is unavailable.",
    "errors.model_not_ready": "The configured model is not ready.",
  });

  const CATALOGS = Object.freeze({
    sv: Object.freeze({
      formatLocale: "sv-SE",
      dir: "ltr",
      messages: sv,
    }),
    en: Object.freeze({
      formatLocale: "en-GB",
      dir: "ltr",
      messages: en,
    }),
  });

  function catalogLanguages(catalogs) {
    const languages = new Map();
    Object.keys(catalogs).forEach((key) => {
      const locale = new Intl.Locale(key);
      languages.set(locale.language, key);
    });
    return languages;
  }

  function assertMatchingCatalogs(catalogs) {
    const catalogKeys = Object.keys(catalogs);
    if (catalogKeys.length === 0) {
      throw new Error("At least one translation catalog is required.");
    }
    const reference = Object.keys(catalogs[catalogKeys[0]].messages).sort();
    catalogKeys.slice(1).forEach((catalogKey) => {
      const candidate = Object.keys(catalogs[catalogKey].messages).sort();
      if (
        reference.length !== candidate.length
        || reference.some((key, index) => key !== candidate[index])
      ) {
        throw new Error(`Translation catalog key mismatch: ${catalogKey}`);
      }
    });
  }

  function resolveLocale(candidates, catalogs = CATALOGS, defaultLocale = "sv") {
    const supported = catalogLanguages(catalogs);
    const fallbackLocale = new Intl.Locale(defaultLocale);
    const fallbackKey = supported.get(fallbackLocale.language) || Object.keys(catalogs)[0];
    const values = Array.isArray(candidates) ? candidates : [candidates];
    for (const candidate of values) {
      if (typeof candidate !== "string" || candidate.length === 0) {
        continue;
      }
      try {
        const locale = new Intl.Locale(candidate);
        const key = supported.get(locale.language);
        if (!key) {
          continue;
        }
        const hasLocaleDetail = Boolean(locale.region || locale.script);
        return Object.freeze({
          locale: key,
          formatLocale: hasLocaleDetail
            ? locale.baseName
            : catalogs[key].formatLocale,
          direction: catalogs[key].dir || "ltr",
        });
      } catch (_error) {
        // Invalid external locale candidates are ignored deliberately.
      }
    }
    return Object.freeze({
      locale: fallbackKey,
      formatLocale: catalogs[fallbackKey].formatLocale,
      direction: catalogs[fallbackKey].dir || "ltr",
    });
  }

  function createI18n(options = {}) {
    const catalogs = options.catalogs || CATALOGS;
    const defaultLocale = options.defaultLocale || "sv";
    const storageKey = options.storageKey || "robot-dashboard-locale";
    const environment = options.environment || global;
    assertMatchingCatalogs(catalogs);

    let persistedLocale = null;
    try {
      persistedLocale = environment.localStorage
        ? environment.localStorage.getItem(storageKey)
        : null;
    } catch (_error) {
      persistedLocale = null;
    }

    const environmentCandidates = [];
    try {
      if (environment.navigator && Array.isArray(environment.navigator.languages)) {
        environmentCandidates.push(...environment.navigator.languages);
      }
      if (environment.navigator && environment.navigator.language) {
        environmentCandidates.push(environment.navigator.language);
      }
    } catch (_error) {
      // Browser locale access is optional.
    }
    try {
      if (environment.document && environment.document.documentElement.lang) {
        environmentCandidates.push(environment.document.documentElement.lang);
      }
    } catch (_error) {
      // The generic module also works without a DOM.
    }

    let resolved = resolveLocale(
      [
        ...(Array.isArray(options.candidates) ? options.candidates : []),
        persistedLocale,
        ...environmentCandidates,
        defaultLocale,
      ],
      catalogs,
      defaultLocale,
    );
    const listeners = new Set();
    const formatterCache = new Map();

    function formatter(kind, formatOptions) {
      const cacheKey = `${resolved.formatLocale}:${kind}:${JSON.stringify(formatOptions || {})}`;
      if (!formatterCache.has(cacheKey)) {
        const Formatter = kind === "number"
          ? Intl.NumberFormat
          : kind === "plural"
            ? Intl.PluralRules
            : Intl.DateTimeFormat;
        formatterCache.set(cacheKey, new Formatter(resolved.formatLocale, formatOptions));
      }
      return formatterCache.get(cacheKey);
    }

    const api = {
      resolve(candidates) {
        return resolveLocale(candidates, catalogs, defaultLocale);
      },
      t(key, args = {}) {
        const messages = catalogs[resolved.locale].messages;
        if (!Object.hasOwn(messages, key)) {
          return key;
        }
        const message = messages[key];
        if (typeof message === "function") {
          return String(message(args, api));
        }
        return typeof message === "string" ? message : key;
      },
      number(value, formatOptions = {}) {
        if (!Number.isFinite(value)) {
          return api.t("common.missing");
        }
        try {
          return formatter("number", formatOptions).format(value);
        } catch (_error) {
          return String(value);
        }
      },
      time(value, formatOptions = {}) {
        const date = value instanceof Date ? value : new Date(value);
        if (!Number.isFinite(date.getTime())) {
          return api.t("common.missing");
        }
        try {
          return formatter("time", formatOptions).format(date);
        } catch (_error) {
          return api.t("common.missing");
        }
      },
      dateTime(value, formatOptions = {}) {
        return api.time(value, formatOptions);
      },
      plural(value, formatOptions = {}) {
        try {
          return formatter("plural", formatOptions).select(value);
        } catch (_error) {
          return "other";
        }
      },
      setLocale(candidate, setOptions = {}) {
        const next = resolveLocale([candidate, defaultLocale], catalogs, defaultLocale);
        const changed = (
          next.locale !== resolved.locale
          || next.formatLocale !== resolved.formatLocale
          || next.direction !== resolved.direction
        );
        resolved = next;
        if (setOptions.persist !== false) {
          try {
            if (environment.localStorage) {
              environment.localStorage.setItem(storageKey, resolved.locale);
            }
          } catch (_error) {
            // A blocked storage API must never block language switching.
          }
        }
        if (changed) {
          formatterCache.clear();
          listeners.forEach((listener) => listener(api));
        }
        return resolved;
      },
      subscribe(listener) {
        listeners.add(listener);
        return () => listeners.delete(listener);
      },
      get locale() {
        return resolved.locale;
      },
      get formatLocale() {
        return resolved.formatLocale;
      },
      get direction() {
        return resolved.direction;
      },
      get supportedLocales() {
        return Object.freeze(Object.keys(catalogs));
      },
    };
    return Object.freeze(api);
  }

  function createDefaultI18n(options = {}) {
    return createI18n({
      ...options,
      catalogs: options.catalogs || CATALOGS,
      defaultLocale: options.defaultLocale || "sv",
      storageKey: options.storageKey || "robot-dashboard-locale",
    });
  }

  global.RobotI18n = Object.freeze({
    CATALOGS,
    createI18n,
    createDefaultI18n,
    resolveLocale,
  });
})(typeof window === "undefined" ? globalThis : window);
