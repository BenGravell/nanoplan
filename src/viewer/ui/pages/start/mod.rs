use super::{PageContext, PageOutput, PageView};

pub(crate) mod landing;
mod track_select;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) enum StartView {
    #[default]
    Menu,
    TrackSelect,
}

#[derive(Default)]
pub(crate) struct StartPage {
    view: StartView,
}

impl StartPage {
    pub(crate) fn select_track(&mut self) {
        self.view = StartView::TrackSelect;
    }

    pub(crate) fn show_menu(&mut self) {
        self.view = StartView::Menu;
    }

    #[cfg(test)]
    pub(crate) fn selecting_track(&self) -> bool {
        self.view == StartView::TrackSelect
    }
}

impl PageView for StartPage {
    fn show(&mut self, context: PageContext<'_>) -> PageOutput {
        let PageContext { root, state, live, .. } = context;
        let route = if self.view == StartView::TrackSelect {
            track_select::show(root, &mut self.view, state, live)
        } else {
            landing::show(root, &mut self.view)
        };
        PageOutput {
            route,
            ..Default::default()
        }
    }
}
