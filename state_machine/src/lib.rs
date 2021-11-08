pub mod descriptor;
pub mod graph_builder;
pub mod graph_printer;
pub mod graph;
pub mod machine;
pub mod state;

#[cfg(test)]
mod tests {
    use std::any::TypeId;

    use anyhow::anyhow;
    use tokio::time::Duration;

    use crate::state::*;
    use crate::machine::*;
    use crate::descriptor::*;
    use crate::graph::*;
    use crate::graph_printer::*;

    const INITIALIZE_DELAY_MS: u64 = 1000;

    #[derive(Default)]
    struct NotInitialized;
    impl State for NotInitialized {
        type Context = TestContext;
        type Event = TestEvent;

        fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
            context.user.event("NotInitialized: enter");
        }

        fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
            context.user.event("NotInitialized: exit");
        }

        fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
            match event {
                TestEvent::DoWork(_, _) => {
                    context.user.event("NotInitialized: DoWork");
                    context.dispatcher().dispatch_sync(TestEvent::DoInitialize);
                    Err(NotHandled::DeferEvent(event))
                }
                _ => Err(NotHandled::UnknownEvent(event)),
            }
        }
    }

    #[derive(Default)]
    struct WaitingToInit;
    impl State for WaitingToInit {
        type Context = TestContext;
        type Event = TestEvent;

        fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
            context.user.event("WaitingToInit: enter");
        }

        fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
            context.user.event("WaitingToInit: exit");
        }

        fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
            match event {
                TestEvent::DoInitialize => {
                    context.user.event("WaitingToInit: DoInitialize");
                    Ok(Transition::MoveTo(TypeId::of::<Initializing>()))
                },
                _ => Err(NotHandled::UnknownEvent(event)),
            }
        }
    }

    #[derive(Default)]
    struct Initializing;
    impl State for Initializing {
        type Context = TestContext;
        type Event = TestEvent;

        fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
            context.user.event("Initializing: enter");
            context.dispatcher().dispatch_delay_ms(TestEvent::OnInitialized, INITIALIZE_DELAY_MS);
        }

        fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
            context.user.event("Initializing: exit");
        }

        fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
            match event {
                TestEvent::DoInitialize => {
                    context.user.event("Initializing: DoInitialize");
                    Ok(Transition::None)
                },
                TestEvent::OnInitialized => {
                    context.user.event("Initializing: OnInitialized");
                    Ok(Transition::MoveTo(TypeId::of::<Ready>()))
                },
                _ => Err(NotHandled::UnknownEvent(event)),
            }
        }
    }

    #[derive(Default)]
    struct Initialized;
    impl State for Initialized {
        type Context = TestContext;
        type Event = TestEvent;

        fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
            context.user.event("Initialized: enter");
        }

        fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
            context.user.event("Initialized: exit");
        }


        fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
            match event {
                _ => Err(NotHandled::UnknownEvent(event)),
            }
        }
    }

    #[derive(Default)]
    struct Ready;
    impl State for Ready {
        type Context = TestContext;
        type Event = TestEvent;

        fn on_enter(&mut self, context: &mut Context<Self::Context, Self::Event>) {
            context.user.event("Ready: enter");
        }

        fn on_exit(&mut self, context: &mut Context<Self::Context, Self::Event>) {
            context.user.event("Ready: exit");
        }


        fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
            match event {
                TestEvent::DoWork(token, will_be_success) => {
                    context.user.event(format!("Ready: DoWork {} {}", token, will_be_success).as_str());
                    let dispatcher = context.dispatcher();
                    if will_be_success {
                        dispatcher.dispatch_sync(TestEvent::OnWorkSuccess);
                    } else {
                        dispatcher.dispatch_sync(TestEvent::OnWorkFailed(anyhow!("Bummer")));
                    }
                    Ok(Transition::MoveTo(TypeId::of::<Busy>()))
                }
                _ => Err(NotHandled::UnknownEvent(event)),
            }
        }
    }

    #[derive(Default)]
    struct Busy;
    impl State for Busy {
        type Context = TestContext;
        type Event = TestEvent;

        fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
            match event {
                TestEvent::OnWorkSuccess => {
                    context.user.event("Busy: OnWorkSuccess");
                    Ok(Transition::MoveTo(TypeId::of::<Ready>()))
                },
                TestEvent::OnWorkFailed(err) => {
                    context.user.event(format!("Busy: OnWorkFailed {:?}", err).as_str());
                    Ok(Transition::MoveTo(TypeId::of::<WaitingToInit>()))
                },
                TestEvent::DoWork(_, _) => {
                    context.user.event("Busy: DoWork");
                    Err(NotHandled::DeferEvent(event))
                },
                _ => Err(NotHandled::UnknownEvent(event)),
            }
        }
    }

    #[derive(Default, Clone)]
    struct TestContext {
        events: Vec<String>,
    }
    impl TestContext {
        fn event(&mut self, label: &str) {
            println!("TestContext: {}", label);
            self.events.push(String::from(label));
        }
    }

    #[derive(Debug)]
    enum TestEvent {
        DoInitialize,
        OnInitialized,
        OnInitializationError(anyhow::Error),
        DoWork(i32, bool),
        OnWorkSuccess,
        OnWorkFailed(anyhow::Error),
    }

    struct TestStateMachine;
    impl StateMachineDescriptor for TestStateMachine {
        type Context = TestContext;
        type Event = TestEvent;

        fn debug_name(&self) -> &'static str {
            return "TestStateMachine";
        }

        fn states(&self) -> StateGraph<Self::Context, Self::Event> {
            StateGraph::builder()
                .add_parent::<NotInitialized>(|s| s
                    .add::<WaitingToInit>().initial().transitions_to::<Initializing>()
                    .add::<Initializing>()
                    .transitions_to::<Ready>()
                    .transitions_to::<WaitingToInit>()
                )
                .add_parent::<Initialized>(|s| s
                    .add::<Ready>().transitions_to::<Busy>()
                    .add::<Busy>().transitions_to::<Ready>())
                .transitions_to::<WaitingToInit>()
                .build()
        }
    }

    #[test]
    fn test_state_builder_smoke() {
        let graph = TestStateMachine {}.states();
        StateGraphPrinter::pretty_print(&graph);
    }

    #[tokio::test]
    async fn test_shutdown_causes_task_cleanup() {
        let machine = StateMachine::<TestContext, TestEvent>::start(TestStateMachine {});
        machine.shutdown().await;
    }

    #[tokio::test(start_paused = true)]
    async fn test_deferred_and_delayed_messages() {
        let machine = StateMachine::<TestContext, TestEvent>::start(TestStateMachine {});
        let dispatcher = machine.dispatcher();
        dispatcher.dispatch(TestEvent::DoWork(123, true)).await;
        dispatcher.dispatch(TestEvent::DoWork(456, false)).await;

        tokio::task::yield_now().await;
        tokio::time::advance(Duration::from_millis(INITIALIZE_DELAY_MS)).await;
        tokio::task::yield_now().await;

        let context = machine.shutdown().await.unwrap();
        assert_eq!(context.events, vec![
            "NotInitialized: enter",
            "WaitingToInit: enter",
            "NotInitialized: DoWork",
            "NotInitialized: DoWork",
            "WaitingToInit: DoInitialize",
            "WaitingToInit: exit",
            "Initializing: enter",
            "NotInitialized: DoWork",
            "NotInitialized: DoWork",
            "Initializing: DoInitialize",
            "Initializing: DoInitialize",
            "Initializing: DoInitialize",
            "Initializing: OnInitialized",
            "Initializing: exit",
            "NotInitialized: exit",
            "Initialized: enter",
            "Ready: enter",
            "Ready: DoWork 123 true",
            "Ready: exit",
            "Busy: DoWork",
            "Busy: OnWorkSuccess",
            "Ready: enter",
            "Ready: DoWork 456 false",
            "Ready: exit",
            "Busy: OnWorkFailed Bummer",
            "Initialized: exit",
            "NotInitialized: enter",
            "WaitingToInit: enter",
        ]);
    }

    #[tokio::test]
    #[should_panic]
    async fn test_invalid_transitions() {
        let machine = StateMachine::<LiteralContext, LiteralEvent>::start(LiteralStateMachine {});
        let dispatcher = machine.dispatcher();
        dispatcher.dispatch(LiteralEvent::GoToZ).await;

        machine.shutdown().await.unwrap();
    }

    #[derive(Default)]
    struct StateX;
    impl State for StateX {
        type Context = LiteralContext;
        type Event = LiteralEvent;

        fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
            common_handling(event)
        }
    }
    #[derive(Default)]
    struct StateY;
    impl State for StateY {
        type Context = LiteralContext;
        type Event = LiteralEvent;

        fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
            common_handling(event)
        }
    }
    #[derive(Default)]
    struct StateZ;
    impl State for StateZ {
        type Context = LiteralContext;
        type Event = LiteralEvent;

        fn handle(&mut self, context: &mut Context<Self::Context, Self::Event>, event: Self::Event) -> HandleResult<Self::Event> {
            common_handling(event)
        }
    }

    fn common_handling(event: LiteralEvent) -> HandleResult<LiteralEvent> {
        match event {
            LiteralEvent::GoToX => {
                Ok(Transition::MoveTo(TypeId::of::<StateX>()))
            },
            LiteralEvent::GoToY => {
                Ok(Transition::MoveTo(TypeId::of::<StateY>()))
            },
            LiteralEvent::GoToZ => {
                Ok(Transition::MoveTo(TypeId::of::<StateZ>()))
            },
            _ => Err(NotHandled::UnknownEvent(event)),
        }
    }

    #[derive(Default)]
    struct LiteralContext;

    #[derive(Debug)]
    enum LiteralEvent {
        GoToX,
        GoToY,
        GoToZ,
    }

    /// A state machine that just does literally whatever the events say, which allows us to invoke
    /// bad runtime behaviour at will.  Eventually we'd like this to all be caught at compile-time
    /// but doing so significantly degrades our ability to implement things without macro hell AFAICT.
    struct LiteralStateMachine;
    impl StateMachineDescriptor for LiteralStateMachine {
        type Context = LiteralContext;
        type Event = LiteralEvent;

        fn debug_name(&self) -> &'static str {
            return "LiteralStateMachine";
        }

        fn states(&self) -> StateGraph<Self::Context, Self::Event> {
            StateGraph::builder()
                .add::<StateX>().initial().transitions_to::<StateY>()
                .add::<StateY>().transitions_to::<StateZ>()
                .add::<StateZ>().transitions_to::<StateX>()
                .build()
        }
    }
}
