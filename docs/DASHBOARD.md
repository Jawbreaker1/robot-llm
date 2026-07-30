# Lokal Mac-dashboard

Dashboarden är en rörelsefri arbetsbänk för Robot LLM Lab. Den fungerar utan
EV3-batterier och samlar lokal textdialog och en lokal röstinmatningspipeline,
read-only research, tekniska händelser, agentbudgetar och ett beskrivande
register över nuvarande och framtida robotkomponenter.

## Starta

LM Studio är valfritt för att öppna gränssnittet, men krävs för att få svar
från Gemma.

```sh
PYTHONPATH=src python3 -m robot_agent.dashboard_cli
```

Öppna sedan den sessionsunika adress som skrivs ut, i formen
`http://127.0.0.1:8765/session/<session-token>/`. Länken är en lokal
bearer-hemlighet: dela den inte. En omstart myntar en ny länk och gör den
gamla ogiltig.

Servern binder alltid till den numeriska loopback-adressen `127.0.0.1`.
Porten kan ändras:

```sh
PYTHONPATH=src python3 -m robot_agent.dashboard_cli --port 8877
```

För att först köra den riktiga 2D-simulatornavigationen och därefter visa dess
ackumulerade karta i dashboarden:

```sh
PYTHONPATH=src python3 -m robot_agent.dashboard_cli \
  --simulation-map-demo
```

Flaggan kontaktar ingen EV3. Utan flaggan visar kartytan ärligt att ingen
kartprovider är ansluten.

### Starta med lokal taligenkänning

Projektets normala röstprofil återanvänder en varm lokal `whisper.cpp`-tjänst
med `large-v3-turbo-q5_0`, som är kvalitetsstandarden för svensk och engelsk
mikrofoninmatning:

```sh
scripts/start_lab_console.sh
```

Profilen ansluter som standard till `http://127.0.0.1:8178/v1`, använder den
separat validerade inferenssökvägen `/audio/transcriptions` och probar tjänsten
innan dashboarden rapporteras som startklar. På den primära utvecklingsdatorn
hålls tjänsten redan varm utanför dashboarden och ska inte startas en gång
till.

På en ny Mac installeras modellen och en kompatibel tjänst så här:

```sh
brew install whisper-cpp
sh scripts/download_whisper_model.sh large-v3-turbo-q5_0
whisper-server \
  --model models/ggml-large-v3-turbo-q5_0.bin \
  --host 127.0.0.1 \
  --port 8178 \
  --threads 8 \
  --language auto \
  --no-timestamps \
  --suppress-nst \
  --request-path /v1 \
  --inference-path /audio/transcriptions
```

Modellen ligger i den git-ignorerade katalogen `models/`. Nedladdningsscriptet
accepterar `large-v3-turbo-q5_0`, `small` och `base` och verifierar den
officiella filens SHA-256 innan den aktiveras. De två mindre modellerna är
endast explicita jämförelsealternativ.

Profilen kan ändras portabelt med `ROBOT_LLM_STT_URL`,
`ROBOT_LLM_STT_INFERENCE_PATH`, `ROBOT_LLM_STT_MODEL_ID` och
`ROBOT_LLM_PYTHON`. Vanliga CLI-argument skickas vidare oförändrade.
Om den externa tjänsten saknas kan dashboarden uttryckligen äga en modell
under sin egen livstid. Stoppa först en konkurrerande GPU-tjänst, eller välj
en isolerad CPU-körning:

```sh
ROBOT_LLM_STT_URL='' scripts/start_lab_console.sh \
  --stt-model models/ggml-large-v3-turbo-q5_0.bin \
  --stt-port 8180
```

Externa STT-adresser måste vara loopback. Bassökvägen måste vara antingen den
exakt tillåtna kompatibilitetssökvägen `/v1` eller ett långt opakt privat
segment. Inferenssökvägen valideras separat som en kanonisk relativ sökväg:
värdnamn, query, fragment, tomma segment, traversal och icke-ASCII-former
nekas. `/v1` är en kompatibilitetsregel, inte en autentiseringshemlighet.

## Språk

Språkväljaren i toppfältet byter hela dashboardens presentationsspråk utan
omladdning eller ny nätverksbegäran. Svenska och engelska är kompletta
kataloger i den första slicen. Det explicita valet sparas lokalt i browsern;
utan ett sparat val används browserns standardsbaserade locale och därefter
svenska som fallback.

Valet skickas även som det typade fältet `response_locale` på varje agenttur.
Fältet valideras genom en exakt allowlist och blir auktoritativt för
`ANSWER`- och `CLARIFY`-texten. Modellen ska alltså inte gissa svarsspråk från
användartext, historik, evidens eller verktygsresultat. Det gör språkbyte
explicit och reproducerbart utan regexp, nyckelord eller språkheuristik.
Hosten kompletterar modellkontexten med locale-namn och en explicit
`response_language_instruction`, upprepar instruktionen på det genererade
textfältets JSON-schema och placerar den sist i den strukturerade kontexten.
Det behövdes i liveprov för att valt språk skulle vinna även över en prompt
och konversationshistorik som uttryckligen begär motsatt språk.

Tekniska identiteter som `weather.current`, eventtyper, ID:n, hashes,
modellnamn och rå JSON översätts aldrig. Etiketter, statusförklaringar,
tillgänglighetstext, placeholders, datum och tider följer däremot vald locale.
Fler språk läggs till som kataloger och tas därefter upp i samma explicita
locale-allowlist; agent- och säkerhetslogiken behöver inte förgrenas per
språk.

## Mikrofon och STT

Talk-knappen är byggd på browserstandarderna `getUserMedia`, Web Audio och
`AudioWorklet`. Den använder inte `SpeechRecognition`, en Safari-specifik
implementation, user-agent-detektering eller någon molnbaserad fallback.

Flödet är:

```text
Talk → nivå/VAD → tystnadsstopp → PCM16-WAV → lokal STT
     → färskt transkript → samma versionskontrollerade agentformulär som text
```

Mikrofoninställningarna ligger separat och sparas endast i browserns lokala
lagring:

- fysisk mikrofon eller systemets standard,
- `auto`, svenska eller engelska som första explicita språkval,
- känslighet med levande nivåmätare och synlig signaltröskel,
- tystnadstid före automatiskt stopp och maximal taltid,
- echo cancellation, noise suppression och automatisk gain,
- om den valda mikrofonströmmen ska hållas varm mellan yttranden,
- automatiskt skicka eller lämna transkriptet redigerbart.

Känsligheten är en deterministisk signaltröskel, inte en semantisk
klassificerare och inte hårdvaruförstärkning. Upp till 1,5 sekunders pre-roll
behålls när tal upptäcks; längre väntetystnad trimmas och mycket korta segment
fylls till backendens minsta säkra WAV-längd. Det ger externa mikrofoners
signalbehandling och en låg första stavelse marginal utan att behålla
obegränsad väntetystnad.

Med **håll mikrofonen redo** aktiverat förblir browserns valda MediaStream och
AudioContext öppna, så browserns mikrofonindikator kan fortsätta vara synlig.
Själva PCM-workleten är däremot stoppad mellan yttranden: den buffrar eller
publicerar inget ljud förrän ett generationsmärkt `start` har skickats och
nollställer bufferten vid både `start` och `stop`. Om inställningen stängs av
frigörs ström, ljudgraf och MessagePort.

Råljud finns i applikationsminnet endast under en aktiv, begränsad inspelning,
medan ett bounded jobb väntar eller medan ett lokalt provideranrop kör. Det
persisteras inte och eventloggen får varken ljud, ljudhash eller transkript.
Transkript har en hostägd leverans-TTL; gamla resultat får inte auto-skickas.
Cancel rensar köat ljud direkt. Ett redan påbörjat provideranrop kan behålla
sin lokala request tills dess deadline, vilket redovisas som
`audio.retained`; dess sena resultat kastas.
Browsern skapar request-ID:t före uppladdningen och kan därför avbryta även
om POST-svaret ännu inte har kommit; en kortlivad, bounded tombstone gör att
en samtidigt anländande uppladdning nekas.

STT-workern är fristående från researchworkern och från robotens framtida
MotionSupervisor. En långsam eller felande transkribering får därför inte
blockera dialog, navigation eller den deterministiska säkerhetsloopen.

## Sex ytor

- **Arbetsbänk** visar en versionsmärkt konversation, pågående tur,
  verifierat slutsvar, typad aktivitet och eventuell evidens.
- **Karta** visar en skrivskyddad snapshot av robotens osäkra lokala
  världsminne: robotpose, färska sensorstrålar, `FREE`/`UNKNOWN`/`OCCUPIED`
  rutor och opaka objekthypoteser med källa, ålder och confidence.
- **Kroppar** visar logiska robotar med controllers och perceptionskällor.
  EV3RSTORM är deklarerad men inte observerad när ingen fysisk probe har
  körts.
- **Händelser** visar den begränsade, append-only eventströmmen med
  korrelations-ID:n. Rå prompt, rå modelltext och fulla evidence-URL:er
  loggas inte.
- **Experiment** reserverar en read-only yta för reproducerbara episoder och
  befintliga experimentartefakter.
- **Inställningar** ändrar sessionsbundna agentbudgetar och browserlokala
  mikrofonval. Agentinställningarna versionskontrolleras och återställs när
  servern startas om; mikrofoninställningarna lämnar aldrig browsern.

## Dialog och kontext

Varje tur har ett explicit läge:

- `conversation` tillåter ett direkt lokalt svar och låter fortfarande
  modellen välja ett godkänt informationsverktyg om färsk data behövs.
- `research_required` kräver verifierad evidens innan modellen får svara.

Hosten klassificerar inte användartext med regexp, substringmatchning eller
nyckelord. Gemma får en typad `conversation_history` med tidigare synliga
user/assistant-meddelanden och returnerar ett strikt `CALL_TOOL`, `ANSWER`,
`CLARIFY` eller `ABORT`. Det enda registrerade externa verktyget i denna
version är `weather.current`.

Dialoghistoriken är ett konversationsminne, inte robotens framtida
världsmodell. Fysiska följdreferenser som “två gånger till” måste senare
bindas till en separat, explicit action-/state-kontext innan någon exekvering
kan bli aktuell.

## Kartvyn

Dashboarden pollar den autentiserade, read-only routen `GET /api/v1/map`.
Resultatet är en frikopplad JSON-snapshot med fast storleksgräns; browsern
får varken kartans skrivcapability eller någon motorcapability.

Den aktuella simulatorn publicerar tre strålar per snapshot, framåt och
`±45°`. Avstånden beskriver robotradie-inflated configuration space och ska
inte tolkas som exakta fysiska objektytor. Kartan fusionerar upprepad positiv
och negativ evidens genom `UNKNOWN`, så en motstridig mätning vänder inte
omedelbart en ruta från upptagen till fri eller tvärtom. Sammanhängande
upptagna rutor blir persistenta, semantiskt opaka `UNKNOWN`-hypoteser. Det
ger en tydlig framtida söm där kamera-, ljud- eller LLM-klassificering kan
lägga till en etikett och evidens utan att ändra den geometriska kartan.

Fysisk EV3 `IR-PROX` är en reflektionssignal och får därför bara visas som
provisorisk kvalitativ evidens. Systemet hittar inte på millimeter, en metrisk
endpoint, objektidentitet eller positivt fri väg. Gamla strålar försvinner ur
livevyn medan kartceller och fortfarande stödda objekthypoteser ligger kvar.
Om den bounded mapping-kön tappar snapshots eller workern får fel markeras
kartan `degraded`; navigationsloopen fortsätter oberoende.

Detta är ännu inte SLAM, global lokalisering, A*, frontier exploration eller
ett navigationsfacit. Kartan är inte återkopplad som motorunderlag i denna
slice. Framtida kameror och flera LEGO-controllers måste publicera explicita
koordinatramar och kalibrerade transformeringar; observationer från olika
ramar får inte slås ihop bara för att de tillhör samma logiska robot.

## Säkerhetsgräns

Dashboardservern har ingen route för:

- motorer eller stoppkommandon
- RobotAPI eller MotionSupervisor
- SSH
- TTS
- uppladdning av events eller robotstatus

Att ett nodkort någon gång visar `online` kommer därför inte att innebära
att noden kan styras från dashboarden.

Webbgränsen använder dessutom:

- en slumpad 256-bitars sessionstoken per serverstart
- tokeniserad bootstrap-/asset-sökväg och tokenkrav på samtliga API-anrop,
  även läsningar
- exakt `Host`- och `Origin`-kontroll
- strikt JSON utan dubblettnycklar eller icke-finita tal
- fasta request- och responsegränser
- en explicit asset- och route-allowlist
- en strikt Content Security Policy utan externa assets
- socket-timeout och ett fast tak för samtidiga HTTP-handlers
- separata begränsade jobbköer för research och STT

Frontend anropar endast hostservern. Den kontaktar aldrig LM Studio,
Open-Meteo eller EV3 direkt.

## Komponentmodell

Registret skiljer på en logisk robot och dess noder:

```text
EV3RSTORM
├── EV3 Main · controller
├── Front Camera · framtida camera source
└── Microphone Array · framtida microphone source

Composite Lab Robot
├── Robot Inventor 51515 · framtida controller
├── BOOST Move Hub · framtida controller
├── Vision Node · framtida camera source
└── Audio Node · framtida microphone source
```

Alla poster är beskrivande och har `control_exposed: false`. Samma kontrakt
kan senare representera flera parallella controllers och perceptionskällor
utan att göra dashboarden till motorägare.
